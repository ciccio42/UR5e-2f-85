#!/usr/bin/env python3
import math
import os
import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import rclpy.wait_for_message
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_TARGET_POSES = (
    '/home/ros2_ws/src/zed_camera/zed_camera_calibration/estimated_camera_positions.yaml'
)


class ArucoCalibrationUntilPose(Node):
    def __init__(self):
        super().__init__('aruco_calibration_until_pose_node')

        self.declare_parameter('cameras_config', '')
        self.declare_parameter('aruco_info', '')
        self.declare_parameter('target_poses', DEFAULT_TARGET_POSES)
        self.declare_parameter('position_tolerance_m', 0.05)
        self.declare_parameter('orientation_tolerance_rad', 0.15)
        self.declare_parameter('stable_detections', 3)
        self.declare_parameter('rolling_window', 5)
        self.declare_parameter('max_attempts_per_camera', 300)
        self.declare_parameter('show_debug_windows', True)
        self.declare_parameter('save_debug_images', True)

        self.position_tolerance = self.get_parameter(
            'position_tolerance_m').get_parameter_value().double_value
        self.orientation_tolerance = self.get_parameter(
            'orientation_tolerance_rad').get_parameter_value().double_value
        self.stable_detections = max(
            1, self.get_parameter('stable_detections').get_parameter_value().integer_value)
        self.rolling_window = max(
            1, self.get_parameter('rolling_window').get_parameter_value().integer_value)
        self.max_attempts_per_camera = max(
            1, self.get_parameter('max_attempts_per_camera').get_parameter_value().integer_value)
        self.show_debug_windows = self.get_parameter(
            'show_debug_windows').get_parameter_value().bool_value
        self.save_debug_images = self.get_parameter(
            'save_debug_images').get_parameter_value().bool_value

        cameras_config_file = self.get_parameter(
            'cameras_config').get_parameter_value().string_value
        aruco_info_file = self.get_parameter(
            'aruco_info').get_parameter_value().string_value
        target_poses_file = self.get_parameter(
            'target_poses').get_parameter_value().string_value

        self.camera_names = self.load_cameras_config(cameras_config_file)
        self.load_aruco_info(aruco_info_file)
        self.target_poses = self.load_target_poses(target_poses_file)
        self.bridge = CvBridge()

    def load_cameras_config(self, path):
        if not path:
            self.get_logger().error('Cameras configuration file not provided.')
            sys.exit(1)
        with open(path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}

        cameras = config.get('cameras', [])
        if not cameras:
            self.get_logger().error(f'No cameras found in configuration: {path}')
            sys.exit(1)

        names = []
        for camera in cameras:
            camera_name = camera.get('camera_name')
            if not camera_name:
                self.get_logger().warning('Skipping camera without camera_name entry.')
                continue
            names.append(camera_name)
            self.get_logger().info(f'Found camera: {camera_name}')
        return names

    def load_aruco_info(self, path):
        if not path:
            self.get_logger().error('ArUco info file not provided.')
            sys.exit(1)
        with open(path, 'r', encoding='utf-8') as f:
            aruco_info = yaml.safe_load(f) or {}

        marker = aruco_info.get('marker_000', {})
        general = aruco_info.get('general', {})
        self.aruco_id = int(marker['aruco_id'])
        self.aruco_pos_map = np.array(marker['position'], dtype=np.float64)
        self.aruco_rpy_map = np.array(marker['orientation'], dtype=np.float64)
        self.aruco_size = float(general['marker_size'])
        self.aruco_dict = self.create_aruco_dictionary()
        self.aruco_params = self.create_aruco_parameters()

    def load_target_poses(self, path):
        if not path:
            self.get_logger().error('Target pose file not provided.')
            sys.exit(1)
        if not os.path.exists(path):
            self.get_logger().error(f'Target pose file does not exist: {path}')
            sys.exit(1)

        with open(path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}

        targets = {}
        for camera_name, pose in raw.items():
            try:
                position = np.array(pose['position'], dtype=np.float64)
                if 'orientation_matrix' in pose:
                    rotation = np.array(pose['orientation_matrix'], dtype=np.float64)
                else:
                    rotation, _ = cv2.Rodrigues(
                        np.array(pose['orientation'], dtype=np.float64).reshape(3, 1))
                targets[camera_name] = {'position': position, 'rotation': rotation}
                self.get_logger().info(
                    f'Loaded target pose for {camera_name}: position={position.tolist()}')
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f'Skipping malformed target pose for {camera_name}: {exc}')

        if not targets:
            self.get_logger().error(f'No valid target poses found in: {path}')
            sys.exit(1)
        return targets

    @staticmethod
    def create_aruco_dictionary():
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)

    @staticmethod
    def create_aruco_parameters():
        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def detect_markers(self, gray):
        # self.get_logger().info('Detecting ArUco markers in the image.')
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.get_logger().info('Calling ArUcoDetector.')
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            return detector.detectMarkers(gray)
        # self.get_logger().info('Calling detectMarkers directly from cv2.aruco.')
        return cv2.aruco.detectMarkers(
            gray, self.aruco_dict)

    def get_camera_intrinsics(self, camera_name):
        topic_name = f'/{camera_name}/zed_node/left/color/rect/camera_info'
        for _ in range(5):
            flag, msg = rclpy.wait_for_message.wait_for_message(
                topic=topic_name,
                msg_type=CameraInfo,
                node=self,
                time_to_wait=2,
            )
            if flag:
                camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
                dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((1, -1))
                self.get_logger().info(
                    f'Camera intrinsics for {camera_name} obtained from {topic_name}.')
                return camera_matrix, dist_coeffs

        self.get_logger().error(f'Failed to receive CameraInfo from topic: {topic_name}')
        sys.exit(1)

    @staticmethod
    def rpy_to_rotation_matrix(rpy):
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return rz @ ry @ rx

    def transform_aruco_to_map(self, rvec, tvec):
        r_ca, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        t_ca = tvec.reshape((3, 1))

        r_ac = r_ca.T
        t_ac = -r_ac @ t_ca

        # The config names this field orientation/rpy, so treat it as roll-pitch-yaw.
        r_am = self.rpy_to_rotation_matrix(self.aruco_rpy_map)
        t_am = self.aruco_pos_map.reshape((3, 1))

        r_cm = r_am @ r_ac
        t_cm = r_am @ t_ac + t_am
        return t_cm.flatten(), r_cm

    @staticmethod
    def average_rotation(rotations):
        quat_outer = np.zeros((4, 4), dtype=np.float64)
        for rotation in rotations:
            quat = ArucoCalibrationUntilPose.rotation_matrix_to_quaternion(rotation)
            quat_outer += np.outer(quat, quat)
        eigvals, eigvecs = np.linalg.eigh(quat_outer)
        quat = eigvecs[:, np.argmax(eigvals)]
        return ArucoCalibrationUntilPose.quaternion_to_rotation_matrix(quat)

    @staticmethod
    def rotation_matrix_to_quaternion(rotation):
        trace = np.trace(rotation)
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (rotation[2, 1] - rotation[1, 2]) / s
            y = (rotation[0, 2] - rotation[2, 0]) / s
            z = (rotation[1, 0] - rotation[0, 1]) / s
        else:
            diag = np.diag(rotation)
            idx = int(np.argmax(diag))
            if idx == 0:
                s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
                w = (rotation[2, 1] - rotation[1, 2]) / s
                x = 0.25 * s
                y = (rotation[0, 1] + rotation[1, 0]) / s
                z = (rotation[0, 2] + rotation[2, 0]) / s
            elif idx == 1:
                s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
                w = (rotation[0, 2] - rotation[2, 0]) / s
                x = (rotation[0, 1] + rotation[1, 0]) / s
                y = 0.25 * s
                z = (rotation[1, 2] + rotation[2, 1]) / s
            else:
                s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
                w = (rotation[1, 0] - rotation[0, 1]) / s
                x = (rotation[0, 2] + rotation[2, 0]) / s
                y = (rotation[1, 2] + rotation[2, 1]) / s
                z = 0.25 * s
        quat = np.array([w, x, y, z], dtype=np.float64)
        return quat / np.linalg.norm(quat)

    @staticmethod
    def quaternion_to_rotation_matrix(quat):
        quat = quat / np.linalg.norm(quat)
        w, x, y, z = quat
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    @staticmethod
    def rotation_error_rad(actual, target):
        delta = target.T @ actual
        cos_angle = (np.trace(delta) - 1.0) / 2.0
        cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
        return math.acos(cos_angle)

    def summarize_window(self, positions, rotations):
        avg_position = np.mean(np.array(positions), axis=0)
        avg_rotation = self.average_rotation(list(rotations))
        return avg_position, avg_rotation

    def estimate_pose_from_image(self, image_cv, camera_matrix, dist_coeffs):
        # self.get_logger().info('Converting image to grayscale for ArUco detection.')
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        # self.get_logger().info('Detecting ArUco markers in the image.')
        corners, ids, _ = self.detect_markers(gray)
        if ids is None:
            self.get_logger().warning('No ArUco markers detected.')
            return None, corners, ids

        ids_flat = ids.flatten().astype(int)
        matches = np.where(ids_flat == self.aruco_id)[0]
        if len(matches) == 0:
            return None, corners, ids

        marker_index = int(matches[0])
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[marker_index]],
            self.aruco_size,
            camera_matrix,
            dist_coeffs,
        )
        rvec = np.asarray(rvecs[0]).reshape(3)
        tvec = np.asarray(tvecs[0]).reshape(3)
        position, rotation = self.transform_aruco_to_map(rvec, tvec)
        return {'position': position, 'rotation': rotation}, corners, ids

    def run_for_camera(self, camera_name):
        target = self.target_poses.get(camera_name)
        if target is None:
            self.get_logger().warning(
                f'No target pose found for {camera_name}; skipping this camera.')
            return None

        image_topic = f'/{camera_name}/zed_node/left/color/rect/image'
        camera_matrix, dist_coeffs = self.get_camera_intrinsics(camera_name)
        positions = deque(maxlen=self.rolling_window)
        rotations = deque(maxlen=self.rolling_window)
        stable_count = 0
        window_name = f'ArUco pose monitor: {camera_name}'

        if self.show_debug_windows:
            try:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                self.get_logger().warning(
                    f'OpenCV window creation failed, continuing headless: {exc}')
                self.show_debug_windows = False

        self.get_logger().info(
            f'Starting pose monitor for {camera_name}. '
            f'Target position={target["position"].tolist()}, '
            f'position_tolerance={self.position_tolerance:.3f} m, '
            f'orientation_tolerance={self.orientation_tolerance:.3f} rad'
            f' with max {self.max_attempts_per_camera} attempts.')
        

        for attempt in range(1, self.max_attempts_per_camera + 1):
            self.get_logger().info(f'[{camera_name}] Waiting for image on {image_topic} (attempt {attempt})...')
            flag, msg = rclpy.wait_for_message.wait_for_message(
                topic=image_topic,
                msg_type=Image,
                node=self,
                time_to_wait=2,
            )
            if not flag:
                self.get_logger().warning(
                    f'[{camera_name}] No image on {image_topic} at attempt {attempt}.')
                continue
            self.get_logger().info(f'[{camera_name}] Received image for attempt {attempt}.')
            image_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #  self.estimate_pose_from_image(
            #     image_cv, camera_matrix, dist_coeffs)
            # self.get_logger().info('Estimating pose from the received image.')
            estimate, corners, ids = self.estimate_pose_from_image(
                 image_cv, camera_matrix, dist_coeffs)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image_cv, corners, ids)

            if estimate is None:
                stable_count = 0
                self.get_logger().warning(
                    f'[{camera_name}] ArUco marker {self.aruco_id} not detected '
                    f'at attempt {attempt}.')
            else:
                positions.append(estimate['position'])
                rotations.append(estimate['rotation'])

                avg_position, avg_rotation = self.summarize_window(positions, rotations)
                pos_error = float(np.linalg.norm(avg_position - target['position']))
                rot_error = self.rotation_error_rad(avg_rotation, target['rotation'])

                if (pos_error <= self.position_tolerance and
                        rot_error <= self.orientation_tolerance):
                    stable_count += 1
                else:
                    stable_count = 0

                self.get_logger().info(
                    f'[{camera_name}] attempt={attempt} samples={len(positions)} '
                    f'pos_error={pos_error:.4f} m rot_error={rot_error:.4f} rad '
                    f'stable={stable_count}/{self.stable_detections}')

                label = (
                    f'{camera_name} pos_err={pos_error:.3f}m '
                    f'rot_err={rot_error:.3f}rad stable={stable_count}/{self.stable_detections}')
                cv2.putText(image_cv, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)

                if self.save_debug_images:
                    output_dir = os.path.join(
                        'src', 'zed_camera', 'zed_camera_calibration', 'calibration_images')
                    os.makedirs(output_dir, exist_ok=True)
                    cv2.imwrite(os.path.join(output_dir, f'{camera_name}_pose_monitor.png'), image_cv)

                if stable_count >= self.stable_detections:
                    self.get_logger().info(
                        f'[{camera_name}] Target pose reached. '
                        f'Final position={avg_position.tolist()}')
                    if self.show_debug_windows:
                        try:
                            cv2.imshow(window_name, image_cv)
                            cv2.waitKey(250)
                            cv2.destroyWindow(window_name)
                        except cv2.error as exc:
                            self.get_logger().warning(
                                f'OpenCV display failed while finishing, continuing: {exc}')
                    return True

            if self.show_debug_windows:
                try:
                    cv2.imshow(window_name, image_cv)
                    if cv2.waitKey(1) in (27, ord('q')):
                        self.get_logger().warning(
                            f'[{camera_name}] Stopped by user key press.')
                        cv2.destroyWindow(window_name)
                        return False
                except cv2.error as exc:
                    self.get_logger().warning(
                        f'OpenCV display failed, continuing headless: {exc}')
                    self.show_debug_windows = False

            time.sleep(0.1)

        self.get_logger().error(
            f'[{camera_name}] Target pose was not reached after '
            f'{self.max_attempts_per_camera} attempts.')
        if self.show_debug_windows:
            try:
                cv2.destroyWindow(window_name)
            except cv2.error:
                pass
        return False

    def run(self):
        self.get_logger().info('ArUco calibration pose monitor is running.')
        results = {}
        for camera_name in self.camera_names:
            results[camera_name] = self.run_for_camera(camera_name)

        passed = [name for name, ok in results.items() if ok is True]
        failed = [name for name, ok in results.items() if ok is False]
        skipped = [name for name, ok in results.items() if ok is None]
        self.get_logger().info(f'Cameras that reached target pose: {passed}')
        if skipped:
            self.get_logger().warning(f'Cameras skipped because no target pose was provided: {skipped}')
        if failed:
            self.get_logger().warning(f'Cameras that did not reach target pose: {failed}')
            return 1
        return 0


def main():
    rclpy.init()
    node = ArucoCalibrationUntilPose()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
