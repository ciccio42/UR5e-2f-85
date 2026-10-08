from __future__ import annotations

from typing import Any, Literal

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation


# =============================================================================
# UR5e / PI0.5 CONSTANTS
# =============================================================================

IMAGE_SIZE = 224

# Convention:
#   (top, bottom, left, right)
#
# Same deterministic front-camera crop used by the existing UR5e pipeline.
FRONT_CROP_MARGINS = (0, 10, 140, 90)

ACTION_DIM = 7
RAW_STATE_DIM = 13

JOINT_POSITION_DIM = 6
EEF_STATE_DIM = 6

# Dataset convention:
#
#   saved_action[:6] = physical_delta[:6] / 0.05
#
# Therefore:
#
#   physical_delta[:6] = saved_action[:6] * 0.05
#
# The gripper component is NOT scaled.
DATASET_ACTION_SCALE = 0.05


# Current dataset action gripper range.
GRIPPER_ACTION_MIN = 0.0
GRIPPER_ACTION_MAX = 20.0

# Hysteresis used to transform the dataset-space gripper prediction into
# the controller binary convention.
#
# These thresholds can later be exposed in the runtime YAML.
GRIPPER_OPEN_THRESHOLD = 5.0
GRIPPER_CLOSE_THRESHOLD = 19.0


# Controller convention.
GRIPPER_OPEN = 0
GRIPPER_CLOSED = 1


# =============================================================================
# COMMON VALIDATION
# =============================================================================


def _as_float_array(
    value,
    expected_shape: tuple[int, ...],
    name: str,
) -> np.ndarray:
    """
    Convert a numerical value to a finite float32 NumPy array and validate
    its shape.
    """

    array = np.asarray(
        value,
        dtype=np.float32,
    )

    if array.shape != expected_shape:
        raise ValueError(
            f"{name} must have shape {expected_shape}, "
            f"got {array.shape}."
        )

    if not np.all(
        np.isfinite(array)
    ):
        raise ValueError(
            f"{name} contains non-finite values: {array}"
        )

    return array


def _validate_uint8_rgb_image(
    image: np.ndarray,
    name: str,
) -> np.ndarray:
    """
    Validate an HWC 3-channel uint8 image.
    """

    image = np.asarray(
        image
    )

    if (
        image.ndim != 3
        or image.shape[2] != 3
    ):
        raise ValueError(
            f"{name} must have shape (H, W, 3), "
            f"got {image.shape}."
        )

    if image.dtype != np.uint8:
        raise TypeError(
            f"{name} must have dtype uint8, "
            f"got {image.dtype}."
        )

    return image


# =============================================================================
# IMAGE PREPROCESSING
# =============================================================================


def _convert_to_rgb(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"],
) -> np.ndarray:
    """
    Explicitly convert the input image to RGB.
    """

    if input_color_order == "rgb":
        return image

    if input_color_order == "bgr":
        return cv2.cvtColor(
            image,
            cv2.COLOR_BGR2RGB,
        )

    raise ValueError(
        f"Unsupported input_color_order: {input_color_order}"
    )


def _rgb_uint8_to_lerobot_tensor(
    image: np.ndarray,
) -> torch.Tensor:
    """
    Convert RGB uint8 HWC image to the visual format supplied to LeRobot:

        dtype = torch.float32
        shape = (3, H, W)
        range = [0, 1]

    No ImageNet normalization or [-1, 1] transformation is performed here.
    Those model-specific operations belong to the LeRobot / PI0.5 pipeline.
    """

    tensor = (
        torch.from_numpy(
            np.ascontiguousarray(image)
        )
        .permute(
            2,
            0,
            1,
        )
        .to(
            dtype=torch.float32
        )
    )

    tensor /= 255.0

    if not torch.isfinite(
        tensor
    ).all():
        raise ValueError(
            "Image tensor contains non-finite values."
        )

    if (
        float(tensor.min()) < 0.0
        or float(tensor.max()) > 1.0
    ):
        raise ValueError(
            "Image tensor values must be in [0, 1]."
        )

    return tensor


def process_front_image(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"] = "rgb",
    crop_margins: tuple[int, int, int, int] = FRONT_CROP_MARGINS,
) -> torch.Tensor:
    """
    Prepare the UR5e front-camera frame.

    Pipeline:

        HWC uint8
            -> RGB conversion if needed
            -> deterministic front crop
            -> resize 224x224
            -> HWC -> CHW
            -> float32 [0,1]

    Returns
    -------
    torch.Tensor
        Shape (3, 224, 224), dtype float32, range [0,1].
    """

    image = _validate_uint8_rgb_image(
        image,
        name="front_image",
    )

    image = _convert_to_rgb(
        image,
        input_color_order=input_color_order,
    )

    top, bottom, left, right = crop_margins

    if min(
        top,
        bottom,
        left,
        right,
    ) < 0:
        raise ValueError(
            f"Crop margins must be non-negative, "
            f"got {crop_margins}."
        )

    height, width = image.shape[:2]

    if top + bottom >= height:
        raise ValueError(
            f"Invalid vertical crop {crop_margins} "
            f"for image height {height}."
        )

    if left + right >= width:
        raise ValueError(
            f"Invalid horizontal crop {crop_margins} "
            f"for image width {width}."
        )

    cropped = image[
        top : height - bottom,
        left : width - right,
    ]

    resized = cv2.resize(
        cropped,
        (IMAGE_SIZE, IMAGE_SIZE),
        interpolation=cv2.INTER_LINEAR,
    )

    tensor = _rgb_uint8_to_lerobot_tensor(
        resized
    )

    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    if tuple(
        tensor.shape
    ) != expected_shape:
        raise RuntimeError(
            "Unexpected front image shape: "
            f"{tuple(tensor.shape)}."
        )

    return tensor


def process_gripper_image(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"] = "rgb",
) -> torch.Tensor:
    """
    Prepare the UR5e gripper-camera frame.

    Unlike the front image, no crop is applied:

        HWC uint8
            -> RGB conversion if needed
            -> resize 224x224
            -> HWC -> CHW
            -> float32 [0,1]

    Returns
    -------
    torch.Tensor
        Shape (3, 224, 224), dtype float32, range [0,1].
    """

    image = _validate_uint8_rgb_image(
        image,
        name="gripper_image",
    )

    image = _convert_to_rgb(
        image,
        input_color_order=input_color_order,
    )

    resized = cv2.resize(
        image,
        (IMAGE_SIZE, IMAGE_SIZE),
        interpolation=cv2.INTER_LINEAR,
    )

    tensor = _rgb_uint8_to_lerobot_tensor(
        resized
    )

    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    if tuple(
        tensor.shape
    ) != expected_shape:
        raise RuntimeError(
            "Unexpected gripper image shape: "
            f"{tuple(tensor.shape)}."
        )

    return tensor


# =============================================================================
# PI0.5 ROBOT STATE
# =============================================================================


def quaternion_to_eef_rpy(
    position_xyz,
    quaternion_xyzw,
) -> np.ndarray:
    """
    Convert the live UR5e EEF pose to the representation used in the
    training dataset:

        [x, y, z, roll, pitch, yaw]

    Rotation convention:
        scipy Rotation.from_quat([qx, qy, qz, qw])
        Euler order = "xyz"

    Returns
    -------
    np.ndarray
        float32, shape (6,)
    """

    position = _as_float_array(
        position_xyz,
        expected_shape=(3,),
        name="eef_position",
    ).astype(
        np.float64
    )

    quaternion = _as_float_array(
        quaternion_xyzw,
        expected_shape=(4,),
        name="eef_quaternion_xyzw",
    ).astype(
        np.float64
    )

    quaternion_norm = float(
        np.linalg.norm(
            quaternion
        )
    )

    if quaternion_norm < 1e-8:
        raise ValueError(
            "EEF quaternion has near-zero norm."
        )

    quaternion /= quaternion_norm

    rpy = (
        Rotation.from_quat(
            quaternion
        )
        .as_euler(
            "xyz",
            degrees=False,
        )
    )

    eef_state = np.concatenate(
        (
            position,
            rpy,
        )
    ).astype(
        np.float32
    )

    if eef_state.shape != (
        EEF_STATE_DIM,
    ):
        raise RuntimeError(
            "Unexpected EEF state shape: "
            f"{eef_state.shape}."
        )

    if not np.all(
        np.isfinite(eef_state)
    ):
        raise ValueError(
            "EEF state contains non-finite values."
        )

    return eef_state


def build_pi05_state(
    joint_positions,
    gripper_state: float,
    eef_position,
    eef_quaternion_xyzw,
) -> torch.Tensor:
    """
    Build the raw 13D ``observation.state`` used by the PI0.5 checkpoint.

    Checkpoint normalization statistics prove that the training state was:

        [
            q1, q2, q3, q4, q5, q6,
            gripper,
            x, y, z, roll, pitch, yaw,
        ]

    Therefore:

        6 joint positions
        + 1 gripper value
        + 6 EEF values
        = 13 dimensions

    The raw state is NOT padded to 32 dimensions here. PI0.5 performs its own
    internal state preparation after checkpoint normalization.

    Parameters
    ----------
    joint_positions:
        Six UR5e joint positions in radians.

    gripper_state:
        Current gripper state in the training-state convention [0,1].

    eef_position:
        Current EEF XYZ position in base_link.

    eef_quaternion_xyzw:
        Current EEF quaternion in ROS/SciPy XYZW convention.

    Returns
    -------
    torch.Tensor
        float32 tensor with shape (13,)
    """

    joints = _as_float_array(
        joint_positions,
        expected_shape=(JOINT_POSITION_DIM,),
        name="joint_positions",
    )

    gripper = float(
        gripper_state
    )

    if not np.isfinite(
        gripper
    ):
        raise ValueError(
            f"Invalid gripper_state: {gripper}"
        )

    # Training state statistics show a gripper state in [0,1].
    if (
        gripper < 0.0
        or gripper > 1.0
    ):
        raise ValueError(
            "PI0.5 state gripper value must be in [0,1], "
            f"got {gripper}."
        )

    eef_state = quaternion_to_eef_rpy(
        position_xyz=eef_position,
        quaternion_xyzw=eef_quaternion_xyzw,
    )

    state = np.concatenate(
        (
            joints,
            np.array(
                [gripper],
                dtype=np.float32,
            ),
            eef_state,
        )
    ).astype(
        np.float32,
        copy=False,
    )

    if state.shape != (
        RAW_STATE_DIM,
    ):
        raise RuntimeError(
            "Unexpected PI0.5 raw state shape: "
            f"{state.shape}."
        )

    if not np.all(
        np.isfinite(state)
    ):
        raise ValueError(
            "PI0.5 raw state contains non-finite values."
        )

    return torch.from_numpy(
        state
    )


# =============================================================================
# RAW LEROBOT OBSERVATION
# =============================================================================


def build_lerobot_observation(
    front_image: torch.Tensor,
    gripper_image: torch.Tensor,
    state: torch.Tensor,
    task: str,
) -> dict[str, Any]:
    """
    Construct the raw observation supplied to PI05Runtime.

    The serialized checkpoint preprocessor later renames:

        observation.images.front
            -> observation.images.base_0_rgb

        observation.images.gripper
            -> observation.images.left_wrist_0_rgb

    This function does NOT:
        - add the batch dimension;
        - move data to CUDA;
        - perform QUANTILES normalization;
        - tokenize the task;
        - pad state to 32 dimensions.

    Those operations belong to the LeRobot checkpoint pipeline.
    """

    expected_image_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    for name, image in (
        (
            "front_image",
            front_image,
        ),
        (
            "gripper_image",
            gripper_image,
        ),
    ):
        if not isinstance(
            image,
            torch.Tensor,
        ):
            raise TypeError(
                f"{name} must be a torch.Tensor."
            )

        if tuple(
            image.shape
        ) != expected_image_shape:
            raise ValueError(
                f"{name} must have shape "
                f"{expected_image_shape}, "
                f"got {tuple(image.shape)}."
            )

        if image.dtype != torch.float32:
            raise TypeError(
                f"{name} must have dtype torch.float32, "
                f"got {image.dtype}."
            )

        if not torch.isfinite(
            image
        ).all():
            raise ValueError(
                f"{name} contains non-finite values."
            )

    if not isinstance(
        state,
        torch.Tensor,
    ):
        raise TypeError(
            "state must be a torch.Tensor."
        )

    if tuple(
        state.shape
    ) != (
        RAW_STATE_DIM,
    ):
        raise ValueError(
            "state must have shape "
            f"({RAW_STATE_DIM},), "
            f"got {tuple(state.shape)}."
        )

    if state.dtype != torch.float32:
        raise TypeError(
            "state must have dtype torch.float32, "
            f"got {state.dtype}."
        )

    if not torch.isfinite(
        state
    ).all():
        raise ValueError(
            "state contains non-finite values."
        )

    if not isinstance(
        task,
        str,
    ):
        raise TypeError(
            f"task must be str, "
            f"got {type(task).__name__}."
        )

    task = task.strip()

    if not task:
        raise ValueError(
            "task must not be empty."
        )

    return {
        "observation.images.front":
            front_image,

        "observation.images.gripper":
            gripper_image,

        "observation.state":
            state,

        "task":
            task,
    }


# =============================================================================
# PI0.5 ACTION VALIDATION
# =============================================================================


def validate_pi05_action(
    action: torch.Tensor | np.ndarray,
) -> np.ndarray:
    """
    Convert the single action returned by PI05Runtime to NumPy float32.

    At this point LeRobot has already applied the serialized QUANTILES
    postprocessor.

    Expected dataset-space action:

        [
            dx,
            dy,
            dz,
            droll,
            dpitch,
            dyaw,
            gripper_action,
        ]

    Shape:
        (7,)
    """

    if isinstance(
        action,
        torch.Tensor,
    ):
        action = (
            action
            .detach()
            .to("cpu")
            .float()
            .numpy()
        )

    else:
        action = np.asarray(
            action,
            dtype=np.float32,
        )

    if action.shape != (
        ACTION_DIM,
    ):
        raise ValueError(
            "PI0.5 action must have shape "
            f"({ACTION_DIM},), "
            f"got {action.shape}."
        )

    if not np.all(
        np.isfinite(action)
    ):
        raise ValueError(
            "PI0.5 action contains non-finite values: "
            f"{action}"
        )

    return action.astype(
        np.float32,
        copy=False,
    )


# =============================================================================
# DATASET ACTION -> PHYSICAL DELTA
# =============================================================================


def recover_physical_delta(
    postprocessed_action: torch.Tensor | np.ndarray,
    scale_factor: float = DATASET_ACTION_SCALE,
) -> np.ndarray:
    """
    Recover the six kinematic components in physical UR5e units.

    The LeRobot postprocessor has already returned the prediction to the
    dataset label space.

    The UR5e dataset stored:

        saved_action[:6] =
            physical_delta[:6] / 0.05

    Therefore inference requires:

        physical_delta[:6] =
            postprocessed_action[:6] * 0.05

    The gripper component is NOT modified here.

    Returns
    -------
    np.ndarray
        [dx, dy, dz, droll, dpitch, dyaw], float32, shape (6,)
    """

    action = validate_pi05_action(
        postprocessed_action
    )

    if (
        not np.isfinite(
            scale_factor
        )
        or scale_factor <= 0.0
    ):
        raise ValueError(
            f"Invalid scale_factor: {scale_factor}"
        )

    physical_delta = (
        action[:6]
        * np.float32(
            scale_factor
        )
    )

    return physical_delta.astype(
        np.float32,
        copy=False,
    )


# =============================================================================
# GRIPPER
# =============================================================================


def gripper_hysteresis(
    gripper_value: float,
    currently_closed: bool,
    open_threshold: float = GRIPPER_OPEN_THRESHOLD,
    close_threshold: float = GRIPPER_CLOSE_THRESHOLD,
) -> tuple[int, bool]:
    """
    Convert the PI0.5 dataset-space gripper action to a binary controller state.

    Checkpoint action statistics show:

        gripper min = 0
        gripper max = 20

    Hysteresis:

        if currently OPEN:
            close only when prediction >= close_threshold

        if currently CLOSED:
            remain closed until prediction < open_threshold

    This prevents rapid open/close oscillations around an ambiguous prediction.

    Returns
    -------
    state:
        0 = open
        1 = closed

    is_closed:
        Boolean equivalent.
    """

    value = float(
        gripper_value
    )

    if not np.isfinite(
        value
    ):
        raise ValueError(
            f"Invalid gripper value: {value}"
        )

    if not (
        GRIPPER_ACTION_MIN
        <= open_threshold
        < close_threshold
        <= GRIPPER_ACTION_MAX
    ):
        raise ValueError(
            "Invalid gripper hysteresis thresholds: "
            f"open={open_threshold}, "
            f"close={close_threshold}."
        )

    if currently_closed:
        is_closed = (
            value >= open_threshold
        )

    else:
        is_closed = (
            value >= close_threshold
        )

    state = (
        GRIPPER_CLOSED
        if is_closed
        else GRIPPER_OPEN
    )

    return (
        int(state),
        bool(is_closed),
    )


def gripper_binary_to_moveit(
    gripper_state: int | bool,
    open_position: float = 0.0,
    closed_position: float = 255.0,
) -> float:
    """
    Convert the common controller convention:

        0 = open
        1 = closed

    to the Robotiq / MoveIt command convention.
    """

    state = int(
        gripper_state
    )

    if state not in (
        GRIPPER_OPEN,
        GRIPPER_CLOSED,
    ):
        raise ValueError(
            "gripper_state must be "
            "0 (open) or 1 (closed), "
            f"got {gripper_state}."
        )

    return float(
        closed_position
        if state == GRIPPER_CLOSED
        else open_position
    )


# =============================================================================
# DELTA ACTION -> ABSOLUTE UR5e TARGET
# =============================================================================


def delta_action_to_absolute_target(
    postprocessed_action: torch.Tensor | np.ndarray,
    reference_position: np.ndarray,
    reference_quaternion_xyzw: np.ndarray,
    currently_closed: bool,
    scale_factor: float = DATASET_ACTION_SCALE,
    open_threshold: float = GRIPPER_OPEN_THRESHOLD,
    close_threshold: float = GRIPPER_CLOSE_THRESHOLD,
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    Convert one postprocessed PI0.5 action to an absolute UR5e target.

    Dataset action:

        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    After recovering the dataset scale:

        delta_p =
            action[:3] * 0.05

        delta_rpy =
            action[3:6] * 0.05

    Translation convention:

        p_target =
            p_current + delta_p

    Rotation convention:

        R_delta =
            Rotation.from_euler(
                "xyz",
                delta_rpy
            )

        R_target =
            R_delta @ R_current

    The left multiplication is important: the dataset analysis established
    that the rotational delta is expressed around the fixed base_link axes.

    Parameters
    ----------
    postprocessed_action:
        7D action already passed through the LeRobot postprocessor.

    reference_position:
        Current real EEF position [x,y,z].

    reference_quaternion_xyzw:
        Current real EEF quaternion [qx,qy,qz,qw].

    currently_closed:
        Current binary gripper state.

    Returns
    -------
    target_position:
        np.float32, shape (3,)

    target_quaternion_xyzw:
        np.float32, shape (4,)

    gripper_state:
        0 = open
        1 = closed
    """

    action = validate_pi05_action(
        postprocessed_action
    )

    physical_delta = recover_physical_delta(
        action,
        scale_factor=scale_factor,
    )

    reference_position = _as_float_array(
        reference_position,
        expected_shape=(3,),
        name="reference_position",
    ).astype(
        np.float64
    )

    reference_quaternion_xyzw = _as_float_array(
        reference_quaternion_xyzw,
        expected_shape=(4,),
        name="reference_quaternion_xyzw",
    ).astype(
        np.float64
    )

    quaternion_norm = float(
        np.linalg.norm(
            reference_quaternion_xyzw
        )
    )

    if quaternion_norm < 1e-8:
        raise ValueError(
            "reference_quaternion_xyzw "
            "has near-zero norm."
        )

    reference_quaternion_xyzw /= (
        quaternion_norm
    )

    # -------------------------------------------------------------------------
    # Translation
    # -------------------------------------------------------------------------

    delta_position = (
        physical_delta[:3]
        .astype(
            np.float64
        )
    )

    target_position = (
        reference_position
        + delta_position
    )

    # -------------------------------------------------------------------------
    # Rotation
    # -------------------------------------------------------------------------

    delta_rpy = (
        physical_delta[3:6]
        .astype(
            np.float64
        )
    )

    current_rotation = (
        Rotation.from_quat(
            reference_quaternion_xyzw
        )
        .as_matrix()
    )

    delta_rotation = (
        Rotation.from_euler(
            "xyz",
            delta_rpy,
            degrees=False,
        )
        .as_matrix()
    )

    # Dataset convention:
    #
    #   R_target = R_delta @ R_current
    #
    target_rotation = (
        delta_rotation
        @ current_rotation
    )

    target_quaternion_xyzw = (
        Rotation.from_matrix(
            target_rotation
        )
        .as_quat()
    )

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------

    gripper_state, _ = gripper_hysteresis(
        gripper_value=float(
            action[6]
        ),
        currently_closed=bool(
            currently_closed
        ),
        open_threshold=open_threshold,
        close_threshold=close_threshold,
    )

    return (
        target_position.astype(
            np.float32
        ),
        target_quaternion_xyzw.astype(
            np.float32
        ),
        gripper_state,
    )