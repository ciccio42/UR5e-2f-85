#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.wait_for_message
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import threading
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from PIL import Image as PILImage

RESOLUTION=(640, 360)


class InteractiveCameraCalibration(Node):
    def __init__(self):
        super().__init__('interactive_aruco_calibration_node')

        # Parameters
        self.declare_parameter('cameras_config', '')
        self.declare_parameter('aruco_info', '')

        # Load cameras configuration file
        cameras_config_file = self.get_parameter('cameras_config').get_parameter_value().string_value
        if not cameras_config_file:
            self.get_logger().error('Cameras configuration file not provided.')
            sys.exit(1)
        self.get_logger().info(f'Loading cameras configuration from: {cameras_config_file}')
        self.load_cameras_config(cameras_config_file)

        # Load aruco info file (not used in this snippet, but declared)
        aruco_info_file = self.get_parameter('aruco_info').get_parameter_value().string_value
        if not aruco_info_file:
            self.get_logger().error('ArUco info file not provided.')
            sys.exit(1)
        self.get_logger().info(f'Loading ArUco info from: {aruco_info_file}')
        self.load_aruco_info(aruco_info_file)
        
        
        self.detection_times = 10  # Number of times to detect the ArUco marker per camera
        self.estimated_pos_dict = {}

    def load_cameras_config(self, cameras_config_file):
        with open(cameras_config_file, 'r') as f:
            cameras_config = yaml.safe_load(f)

        self.camera_names = []
        for camera in cameras_config['cameras']:
            camera_name = camera['camera_name']
            self.get_logger().info(f'Found camera: {camera_name}')
            self.camera_names.append(camera_name)

    def load_aruco_info(self, aruco_info_file):
        with open(aruco_info_file, 'r') as f:
            aruco_info = yaml.safe_load(f)
        self.aruco_id = aruco_info['marker_000']['aruco_id']
        self.aruco_pos_map = np.array(aruco_info['marker_000']['position'])
        self.aruco_rpy_map = np.array(aruco_info['marker_000']['orientation'])

        self.aruco_size = aruco_info['general']['marker_size']
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)

    def get_camera_intrinsics(self, camera_name):
        
        topic_name = f'/{camera_name}/zed_node/left/color/rect/camera_info'
        flag = False
        cnt = 5
        while not flag and cnt > 0:
            flag, msg = rclpy.wait_for_message.wait_for_message(
                topic=topic_name,
                msg_type=CameraInfo,
                node=self,
                time_to_wait=2
            )
            cnt -= 1

        if not flag:
            self.get_logger().error(f'Failed to receive CameraInfo from topic: {topic_name}')
            sys.exit(1)
        
        camera_matrix = cv2.UMat(3, 3, cv2.CV_64F)
        dist_coeffs = cv2.UMat(1, 5, cv2.CV_64F)

        camera_matrix = cv2.UMat(np.array(msg.k).reshape((3, 3)))
        dist_coeffs = cv2.UMat(np.array(msg.d).reshape((1, 5)))
        self.get_logger().info(f'Camera intrinsics for {camera_name} obtained.')
        self.get_logger().info(f'Camera Matrix:\n{camera_matrix.get()}\nDistortion Coefficients:\n{dist_coeffs.get()}')

        return camera_matrix, dist_coeffs

    def transform_aruco_to_map(self, rvec, tvec):
        # Convert rotation vector to rotation
        R_ca, _ = cv2.Rodrigues(rvec)
        t_ca = tvec.reshape((3, 1))
        # Invert to get camera to ArUco
        R_ac = R_ca.T
        t_ac = -R_ac @ t_ca
        # ArUco to Map
        R_am, _ = cv2.Rodrigues(self.aruco_rpy_map)
        t_am = self.aruco_pos_map.reshape((3, 1))
        R_cm = R_am @ R_ac
        t_cm = R_am @ t_ac + t_am
        rvec_cm, _ = cv2.Rodrigues(R_cm)
        return t_cm.flatten(), rvec_cm.flatten()

    def run(self):
        self.get_logger().info('Interactive ArUco Calibration Node is running.')
        

        for camera_name in self.camera_names:
            if camera_name not in self.estimated_pos_dict.keys():
                self.estimated_pos_dict[camera_name] = {}

            self.get_logger().info(f'Starting calibration for camera: {camera_name}')

            image_topic = f'/{camera_name}/zed_node/left/color/rect/image'
            self.get_logger().info(f'\tSubscribing to image topic: {image_topic}')
            
            self.estimated_position_list = []
            self.estimated_orientation_list = []

            bridge = CvBridge()
            # FIXED WINDOW NAME
            window_name = f'Camera: {camera_name}'
            aruco_window_name = f'ArUco Detection: {camera_name}'
            overlapped_image_window_name = f'Overlapped Image: {camera_name}'
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.namedWindow(aruco_window_name, cv2.WINDOW_NORMAL)
            cv2.namedWindow(overlapped_image_window_name, cv2.WINDOW_NORMAL)

            camera_matrix, dist_coeffs = self.get_camera_intrinsics(camera_name)

            if "front" in camera_name.lower():
                traj_name = "traj000_camera_front_image.png"
            elif "left" in camera_name.lower():
                traj_name = "traj000_camera_lateral_left_image.png"
            elif "right" in camera_name.lower():
                traj_name = "traj000_camera_lateral_right_image.png"
            frane_path = f"src/zed_camera/zed_camera_calibration/{traj_name}"
            # # take the first frame from video
            # cap = cv2.VideoCapture(video_path)
            # ret, frame = cap.read()
            # if not ret:
            #     self.get_logger().error(f'Failed to read video file: {video_path}')
            #     exit(1)
            # load image
            frame = cv2.imread(frane_path)

            if frame.shape[0] != 360 or frame.shape[1] != 640:
                # crop the center of the image to 640x360
                x_center = RESOLUTION[0] // 2
                y_center = RESOLUTION[1] // 2
                x_start = (frame.shape[1] // 2) - x_center  
                y_start = (frame.shape[0] // 2) - y_center
                frame = frame[y_start:y_start + RESOLUTION[1], x_start:x_start + RESOLUTION[0]]

            for i in range(self.detection_times):
            #while True:
                flag, msg = rclpy.wait_for_message.wait_for_message(topic=image_topic,
                                                                    msg_type=Image,
                                                                    node=self,
                                                                    time_to_wait=2)
                if not flag:
                    self.get_logger().error(f'Failed to receive image from topic: {image_topic}')
                    continue

                
                image_cv = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # create an overlapped image between image_cv and frame
                self.get_logger().info(f'Shape current frame from video: {frame.shape}, Shape image from topic: {image_cv.shape}')
                alpha = 0.5
                overlapped_image = cv2.addWeighted(image_cv, alpha, frame, 1 - alpha, 0)
                # show the overlapped image
                cv2.imshow(overlapped_image_window_name, overlapped_image)


                # save image
                pil_img = PILImage.fromarray(cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB))
                os.makedirs(os.path.join('src', 'zed_camera', 'zed_camera_calibration','calibration_images'), exist_ok=True)
                img_save_path = os.path.join('src', 'zed_camera', 'zed_camera_calibration','calibration_images', f'{camera_name}_image.png')
                pil_img.save(img_save_path)

                cv2.putText(
                    image_cv,
                    f'Detection', #{i+1}/{self.detection_times}',
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2
                )   
                
                cv2.imshow(window_name, image_cv)
  
                # Perform ArUco detection here
                gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)

                # estimate pose if detected
                if ids is not None and self.aruco_id in ids:
                    self.get_logger().info(f'\tArUco marker {self.aruco_id} detected in camera {camera_name}.')
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners,
                        self.aruco_size,
                        camera_matrix,
                        dist_coeffs
                    )
                    # Convert to numpy arrays
                    rvecs = rvecs.get() if isinstance(rvecs, cv2.UMat) else rvecs
                    tvecs = tvecs.get() if isinstance(tvecs, cv2.UMat) else tvecs

                    # Now iterate safely
                    cv2.aruco.drawDetectedMarkers(image_cv, corners, ids)
                    for rvec, tvec in zip(rvecs, tvecs):
                        self.get_logger().info(f'\t\tEstimated rvec: {rvec.flatten()}')
                        self.get_logger().info(f'\t\tEstimated tvec: {tvec.flatten()}')
                        # cv2.drawFrameAxes(
                        #     image_cv,
                        #     camera_matrix,
                        #     dist_coeffs,
                        #     rvec,
                        #     tvec,
                        #     10.0,
                        #     thickness=10
                        # )
                        
                        # From 

                        # convert from Camera frame to Map frame (assuming ArUco is at known position in Map frame)
                        tvec_map, rvec_map = self.transform_aruco_to_map(rvec, tvec)
                        self.get_logger().info(f'\t\tTransformed to Map frame - Position: {tvec_map}, Orientation: {rvec_map}')

                        self.estimated_position_list.append(tvec_map.flatten())
                        self.estimated_orientation_list.append(rvec_map.flatten())

                else:
                    self.get_logger().error(f'\tCannot estimate pose for marker {self.aruco_id} as it was not detected.')


                # Show the image with detected markers
                cv2.imshow(aruco_window_name, image_cv)
                # wait for enter key
                # self.get_logger().info(f'\tPress ENTER to capture image {i+1}/{self.detection_times} for camera {camera_name}...')
                # cv2.waitKey(0)  # Wait indefinitely for a key press
                # wait for 1 second before next capture
                self.get_logger().info(f'\tWaiting 1 second before next capture for camera {camera_name}...')
                key = cv2.waitKey(1000)  # Wait for 1 second

            # Average position and orientation
            if self.estimated_position_list and self.estimated_orientation_list:
                avg_position = np.mean(self.estimated_position_list, axis=0)
                avg_orientation = np.mean(self.estimated_orientation_list, axis=0)

                self.estimated_pos_dict[camera_name]['position'] = avg_position.tolist()
                self.estimated_pos_dict[camera_name]['orientation'] = avg_orientation.tolist()
                self.estimated_pos_dict[camera_name]['orientation_matrix'] = cv2.Rodrigues(avg_orientation)[0].tolist()

                self.get_logger().info(f'Estimated position for camera {camera_name}: {avg_position}')
                self.get_logger().info(f'Estimated orientation for camera {camera_name}: {avg_orientation}')
            else:
                self.get_logger().error(f'No valid ArUco detections were made for camera {camera_name}.')

            # ---- CLOSE WINDOW ONCE ----
            cv2.destroyWindow(window_name)
            cv2.destroyWindow(aruco_window_name)

        # get current package path
        self.get_logger().info('Calibration completed for all cameras.')
        save_path = os.path.join('src', 'zed_camera', 'zed_camera_calibration','estimated_camera_positions.yaml')
        with open(save_path, 'w') as f:
            yaml.dump(self.estimated_pos_dict, f)


        return 0




if __name__ == '__main__':
    
    rclpy.init()

    node = InteractiveCameraCalibration()

    thread = threading.Thread(target=node.run)
    thread.start()

    rclpy.spin(node)

    node.destroy_node()
