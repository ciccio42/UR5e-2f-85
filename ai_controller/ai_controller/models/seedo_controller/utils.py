import threading
import time
import numpy as np
import rclpy
import tf2_ros
import cv2
import math

from ai_controller.utils.utils import _quat2mat



def wait_for_future(future, timeout_sec=None):
        done_event = threading.Event()

        future.add_done_callback(
            lambda _: done_event.set()
        )

        if not done_event.wait(timeout=timeout_sec):
            return None

        return future.result()

def lookup_transform(
    node,
    target_frame,
    source_frame,
    max_attempts=100,
):
    last_exc = None

    for attempt in range(1, max_attempts + 1):
        try:
            return node.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            last_exc = exc

            node.get_logger().warning(
                f'lookup_transform('
                f'{target_frame} -> {source_frame}) failed '
                f'(attempt {attempt}/{max_attempts}): {exc}'
            )

            if node.ai_controller_target == 'seedo_controller':
                time.sleep(0.1)
            else:
                rclpy.spin_once(
                    node,
                    timeout_sec=0.1,
                )

    raise RuntimeError(
        f'Failed to look up transform '
        f'{target_frame} -> {source_frame} after '
        f'{max_attempts} attempts: {last_exc}'
    )

def get_seedo_base_to_table_transform(node):
    """Return the current table_0 -> base_link transform in SeeDo format.

    The returned transform maps points expressed in table_0 coordinates
    into base_link coordinates:

        p_base = R_base_table @ p_table + t_base_table

    Returns
    -------
    dict
        {
            "rotation": np.ndarray shape (3, 3),
            "translation": np.ndarray shape (3,)
        }
    """
    transform = lookup_transform(
        node,
        node.frame_id,          # base_link
        node.seedo_table_frame  # table_0
    )

    t = transform.transform.translation
    q = transform.transform.rotation

    quat = np.array(
        [q.x, q.y, q.z, q.w],
        dtype=np.float64,
    )

    return {
        "rotation": _quat2mat(quat),
        "translation": np.array(
            [t.x, t.y, t.z],
            dtype=np.float64,
        ),
    }

def compress_seedo_dataset_rgb(
    camera_data,
):
    """
    JPEG-compress only the RGB images stored in the SeeDo
    dataset rollout. This does not affect legacy controllers.
    """

    rgb_keys = (
        "camera_front_image",
        "camera_lateral_left_image",
        "camera_lateral_right_image",
        "eye_in_hand_image",
    )

    compressed = dict(
        camera_data
    )

    for key in rgb_keys:
        if key not in compressed:
            raise RuntimeError(
                f"Missing SeeDo rollout RGB image: {key}"
            )

        image = np.asarray(
            compressed[key]
        )

        if image.ndim != 3 or image.shape[2] != 3:
            raise RuntimeError(
                f"Invalid image shape for {key}: "
                f"{image.shape}"
            )

        okay, encoded = cv2.imencode(
            ".jpg",
            image,
        )

        if not okay:
            raise RuntimeError(
                f"JPEG encoding failed for {key}"
            )

        compressed[key] = encoded

    return compressed


def build_seedo_front_obj_bb(
    perception_result,
    scene_state,
):
    """
    Build the reference-dataset obj_bb structure from the
    runtime ScenePerceiver detections.

    Bounding boxes are derived from the SAM masks produced
    on the live front-camera scene.
    """

    if perception_result is None:
        raise RuntimeError(
            "SeeDo perception_result is not available."
        )

    if scene_state is None:
        raise RuntimeError(
            "SeeDo scene_state is not available."
        )

    raw_objects = (
        perception_result
        .raw_scene
        .objects
    )

    semantic_objects = (
        scene_state.objects
    )

    if len(raw_objects) != len(semantic_objects):
        raise RuntimeError(
            "Raw and semantic scene object counts differ: "
            f"{len(raw_objects)} != "
            f"{len(semantic_objects)}"
        )

    semantic_to_dataset_name = {
        "red cube": "redbox",
        "green cube": "greenbox",
        "blue cube": "bluebox",
        "yellow cube": "yellowbox",

        "first bin from the left": "bin_0",
        "second bin from the left": "bin_1",
        "third bin from the left": "bin_2",
        "fourth bin from the left": "bin_3",
    }

    camera_front_bb = {}

    for raw_object, semantic_object in zip(
        raw_objects,
        semantic_objects,
    ):
        semantic_name = (
            semantic_object.object_id
            .strip()
            .lower()
        )

        if semantic_name in semantic_to_dataset_name:
            dataset_name = semantic_to_dataset_name[
                semantic_name
            ]
        else:
            # Generic semantic objects introduced by the new
            # ScenePerceiver, e.g.:
            #
            #   "blue ring"      -> "blue_ring"
            #   "red cylinder"   -> "red_cylinder"
            #   "green star"     -> "green_star"
            #
            # The four legacy cubes and storage bins keep their
            # original dataset-compatible names through the map above.
            dataset_name = (
                semantic_name
                .replace(" ", "_")
            )

        mask = raw_object.mask

        if mask is None:
            raise RuntimeError(
                f"Object {semantic_name!r} has no SAM mask."
            )

        mask = np.asarray(
            mask,
            dtype=bool,
        )

        ys, xs = np.where(mask)

        if xs.size == 0 or ys.size == 0:
            raise RuntimeError(
                f"Object {semantic_name!r} has an empty SAM mask."
            )

        x_min = int(xs.min())
        x_max = int(xs.max())

        y_min = int(ys.min())
        y_max = int(ys.max())

        center_x = int(
            np.rint(
                (x_min + x_max) / 2.0
            )
        )

        center_y = int(
            np.rint(
                (y_min + y_max) / 2.0
            )
        )

        camera_front_bb[
            dataset_name
        ] = {
            "upper_left_corner": [
                x_max,
                y_max,
            ],
            "bottom_right_corner": [
                x_min,
                y_min,
            ],
            "center": [
                center_x,
                center_y,
            ],
        }

    return {
        "camera_front": camera_front_bb
    }

def get_seedo_dataset_status(
    primitive_name,
    is_final_action=False,
):
    """
    Map a SeeDo primitive to the trajectory status used
    by the reference robot dataset.
    """

    if is_final_action:
        return "end"

    status_map = {
        "reach": "start",
        "approaching": "approaching",
        "pick": "picking",
        "lift_up": "picking",
        "moving": "moving",
        "placing": "placing",
    }

    primitive_name = str(
        primitive_name
    ).strip().lower()

    if primitive_name not in status_map:
        raise RuntimeError(
            "Unsupported SeeDo primitive for dataset status: "
            f"{primitive_name!r}"
        )

    return status_map[
        primitive_name
    ]

def get_seedo_artifacts_dir(artifacts_dir):
    if not artifacts_dir.strip():
        return None

    return artifacts_dir

def wait_for_seedo_runtime_data(
    node,
    timeout: float = 10.0,
) -> None:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        rgbd_ready = node.seedo_rgbd_event.is_set()
        camera_info_ready = node.seedo_camera_info_msg is not None

        if rgbd_ready and camera_info_ready:
            return

        time.sleep(0.01)

    if not node.seedo_rgbd_event.is_set():
        raise TimeoutError(
            "Timed out waiting for synchronized SeeDo RGB-D data."
        )

    raise TimeoutError(
        "Timed out waiting for SeeDo CameraInfo data."
    )

def build_seedo_runtime_input(
    rgb_image,
    depth_image,
    camera_info,
    base_to_table_transform,
):
    if rgb_image is None:
        raise ValueError(
            "SeeDo runtime input is missing the RGB image."
        )

    if depth_image is None:
        raise ValueError(
            "SeeDo runtime input is missing the depth image."
        )

    if camera_info is None:
        raise ValueError(
            "SeeDo runtime input is missing camera_info."
        )

    if base_to_table_transform is None:
        raise ValueError(
            "SeeDo runtime input is missing "
            "base_to_table_transform."
        )

    return {
        "rgb": rgb_image,
        "depth": depth_image,
        "camera_info": camera_info,
        "base_to_table_transform": base_to_table_transform,
    }

def get_seedo_runtime_input(
    node,
    base_to_table_transform,
):
    if node.seedo_rgb_msg is None:
        raise RuntimeError(
            "No synchronized SeeDo RGB frame is available."
        )

    if node.seedo_depth_msg is None:
        raise RuntimeError(
            "No synchronized SeeDo depth frame is available."
        )

    if node.seedo_camera_info_msg is None:
        raise RuntimeError(
            "No SeeDo CameraInfo message is available."
        )

    rgb_image = node.bridge.imgmsg_to_cv2(
        node.seedo_rgb_msg,
        desired_encoding="rgb8",
    )

    depth_image = node.bridge.imgmsg_to_cv2(
        node.seedo_depth_msg,
        desired_encoding="passthrough",
    )

    camera_info_msg = node.seedo_camera_info_msg

    camera_info = {
        "height": camera_info_msg.height,
        "width": camera_info_msg.width,
        "distortion_model": camera_info_msg.distortion_model,
        "d": list(camera_info_msg.d),
        "k": list(camera_info_msg.k),
        "r": list(camera_info_msg.r),
        "p": list(camera_info_msg.p),
    }

    return build_seedo_runtime_input(
        rgb_image=rgb_image,
        depth_image=depth_image,
        camera_info=camera_info,
        base_to_table_transform=base_to_table_transform,
    )

def quat2axisangle(quat):
    """
    Convert quaternion [x, y, z, w] to axis-angle representation.
    """

    quat = np.asarray(
        quat,
        dtype=np.float64,
    ).copy()

    quat[3] = np.clip(
        quat[3],
        -1.0,
        1.0,
    )

    den = np.sqrt(
        1.0 - quat[3] * quat[3]
    )

    if math.isclose(
        den,
        0.0,
        abs_tol=1e-8,
    ):
        return np.zeros(
            3,
            dtype=np.float64,
        )

    return (
        quat[:3]
        * 2.0
        * math.acos(quat[3])
        / den
    )

def gripper_joint_position_to_raw(
    joint_position,
):
    """
    Convert the Robotiq joint position in radians to the
    raw representation used by the reference dataset.
    """

    joint_position = float(
        np.asarray(
            joint_position
        ).reshape(-1)[0]
    )

    raw_open = 3
    raw_closed = 230

    joint_open = 0.0
    joint_closed = 0.8

    ratio = (
        (joint_position - joint_open)
        / (joint_closed - joint_open)
    )

    ratio = np.clip(
        ratio,
        0.0,
        1.0,
    )

    raw_position = (
        raw_open
        + ratio * (
            raw_closed - raw_open
        )
    )

    return int(
        np.rint(
            raw_position
        )
    )