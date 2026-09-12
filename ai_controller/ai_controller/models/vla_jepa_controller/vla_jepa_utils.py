from __future__ import annotations

from typing import Any, Literal

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation


# =============================================================================
# CONFIGURAZIONE UR5e / VLA-JEPA
# =============================================================================

IMAGE_SIZE = 224

# Convenzione:
#   (top, bottom, left, right)
#
# Stesso crop utilizzato per la camera frontale del dataset UR5e.
FRONT_CROP_MARGINS = (0, 10, 140, 90)

ACTION_DIM = 7

# Nel dataset UR5e le prime sei componenti della action sono state salvate
# dividendo il delta fisico per SCALE_FACTOR.
#
# In inference, dopo il postprocessor LeRobot, occorre quindi riportarle
# alle unità fisiche:
#
#   physical_delta = dataset_action * DATASET_ACTION_SCALE
#
# Il gripper NON viene moltiplicato per questo fattore.
DATASET_ACTION_SCALE = 0.05

# Convenzione interna ai controller:
GRIPPER_OPEN = 0
GRIPPER_CLOSED = 1


# =============================================================================
# VALIDAZIONE COMUNE
# =============================================================================


def _as_float_array(
    value,
    expected_shape: tuple[int, ...],
    name: str,
) -> np.ndarray:
    """
    Converte un valore numerico in NumPy float32 e ne verifica la shape.
    """
    array = np.asarray(value, dtype=np.float32)

    if array.shape != expected_shape:
        raise ValueError(
            f"{name} must have shape {expected_shape}, "
            f"got {array.shape}"
        )

    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"{name} contains non-finite values: {array}"
        )

    return array


def _validate_uint8_rgb_image(
    image: np.ndarray,
    name: str,
) -> np.ndarray:
    """
    Verifica che un'immagine sia HWC, 3 canali, uint8.
    """
    image = np.asarray(image)

    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(
            f"{name} must have shape (H, W, 3), "
            f"got {image.shape}"
        )

    if image.dtype != np.uint8:
        raise TypeError(
            f"{name} must have dtype uint8, got {image.dtype}"
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
    Converte esplicitamente l'immagine nel formato RGB.
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
    Converte una immagine RGB uint8 HWC nel formato visuale utilizzato
    da VLA-JEPA:

        torch.float32
        shape = (3, H, W)
        range = [0, 1]

    Non applica ImageNet normalization o altre trasformazioni:
    il checkpoint dichiara VISUAL -> IDENTITY.
    """
    tensor = torch.from_numpy(
        np.ascontiguousarray(image)
    ).permute(
        2,
        0,
        1,
    ).to(
        dtype=torch.float32
    )

    tensor /= 255.0

    if not torch.isfinite(tensor).all():
        raise ValueError(
            "Image tensor contains non-finite values."
        )

    if tensor.min() < 0.0 or tensor.max() > 1.0:
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
    Prepara il frame corrente della camera frontale.

    Pipeline:

        HWC uint8
            -> BGR/RGB conversione se necessaria
            -> crop globale della scena
            -> resize 224x224
            -> HWC -> CHW
            -> float32 [0,1]

    Il crop replica quello utilizzato per il dataset UR5e:

        (top, bottom, left, right)
        (0, 10, 140, 90)

    Returns:
        torch.float32, shape (3, 224, 224), range [0,1].
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

    if min(top, bottom, left, right) < 0:
        raise ValueError(
            f"Crop margins must be non-negative, got {crop_margins}"
        )

    height, width = image.shape[:2]

    if top + bottom >= height:
        raise ValueError(
            f"Invalid vertical crop {crop_margins} "
            f"for image height {height}"
        )

    if left + right >= width:
        raise ValueError(
            f"Invalid horizontal crop {crop_margins} "
            f"for image width {width}"
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

    if tuple(tensor.shape) != expected_shape:
        raise RuntimeError(
            f"Unexpected front image shape: "
            f"{tuple(tensor.shape)}"
        )

    return tensor


def process_gripper_image(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"] = "rgb",
) -> torch.Tensor:
    """
    Prepara il frame corrente della camera sul gripper.

    A differenza della camera frontale NON viene applicato alcun crop:

        HWC uint8
            -> BGR/RGB conversione se necessaria
            -> resize 224x224
            -> HWC -> CHW
            -> float32 [0,1]

    Returns:
        torch.float32, shape (3, 224, 224), range [0,1].
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

    if tuple(tensor.shape) != expected_shape:
        raise RuntimeError(
            f"Unexpected gripper image shape: "
            f"{tuple(tensor.shape)}"
        )

    return tensor


def build_lerobot_observation(
    front_image: torch.Tensor,
    gripper_image: torch.Tensor,
    task: str,
) -> dict[str, Any]:
    """
    Costruisce l'osservazione raw da passare a VLAJEPARuntime.

    Il preprocessor serializzato nel checkpoint rinominerà:

        observation.images.front
            -> observation.images.exterior_1_left

        observation.images.gripper
            -> observation.images.exterior_2_left

    Questa funzione NON:
        - aggiunge batch dimension;
        - sposta i tensori su CUDA;
        - normalizza le immagini.

    Queste operazioni appartengono alla pipeline LeRobot.
    """
    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    for name, image in (
        ("front_image", front_image),
        ("gripper_image", gripper_image),
    ):
        if not isinstance(image, torch.Tensor):
            raise TypeError(
                f"{name} must be a torch.Tensor."
            )

        if tuple(image.shape) != expected_shape:
            raise ValueError(
                f"{name} must have shape {expected_shape}, "
                f"got {tuple(image.shape)}"
            )

        if image.dtype != torch.float32:
            raise TypeError(
                f"{name} must have dtype torch.float32, "
                f"got {image.dtype}"
            )

    if not isinstance(task, str):
        raise TypeError(
            f"task must be str, got {type(task).__name__}"
        )

    task = task.strip()

    if not task:
        raise ValueError(
            "task must not be empty."
        )

    return {
        "observation.images.front": front_image,
        "observation.images.gripper": gripper_image,
        "task": task,
    }


# =============================================================================
# ACTION POSTPROCESSING
# =============================================================================


def validate_vla_jepa_action(
    action: torch.Tensor | np.ndarray,
) -> np.ndarray:
    """
    Converte la singola action restituita da VLAJEPARuntime in NumPy
    float32 e verifica:

        shape == (7,)
        valori finiti

    A questo punto LeRobot ha già applicato il proprio postprocessor.
    """
    if isinstance(action, torch.Tensor):
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

    if action.shape != (ACTION_DIM,):
        raise ValueError(
            f"VLA-JEPA action must have shape ({ACTION_DIM},), "
            f"got {action.shape}"
        )

    if not np.all(np.isfinite(action)):
        raise ValueError(
            f"VLA-JEPA action contains non-finite values: {action}"
        )

    return action.astype(
        np.float32,
        copy=False,
    )


def recover_physical_delta(
    postprocessed_action: torch.Tensor | np.ndarray,
    scale_factor: float = DATASET_ACTION_SCALE,
) -> np.ndarray:
    """
    Recupera le sei componenti cinematiche nelle unità fisiche UR5e.

    Il postprocessor LeRobot ha già effettuato la denormalizzazione
    MIN_MAX nello spazio delle label salvate nel dataset.

    Le label UR5e erano tuttavia state scalate durante la costruzione
    del dataset. Per recuperare i delta fisici:

        [dx, dy, dz, droll, dpitch, dyaw]
            = action[:6] * scale_factor

    Il gripper NON viene modificato da questa funzione.

    Returns:
        np.float32, shape (6,).
    """
    action = validate_vla_jepa_action(
        postprocessed_action
    )

    if not np.isfinite(scale_factor) or scale_factor <= 0.0:
        raise ValueError(
            f"Invalid scale_factor: {scale_factor}"
        )

    return (
        action[:6] * np.float32(scale_factor)
    ).astype(
        np.float32,
        copy=False,
    )


def decode_gripper_binary(
    postprocessed_gripper: float,
    tolerance: float = 0.25,
) -> int:
    """
    Converte il gripper prodotto dal postprocessor VLA-JEPA nella
    convenzione comune dei controller:

        0 = OPEN
        1 = CLOSED

    Il postprocessor VLA-JEPA binarizza normalmente il gripper in:

        +1 -> OPEN
        -1 -> CLOSED

    quindi:

        controller_value = 0 se gripper > 0
        controller_value = 1 se gripper < 0

    La tolleranza serve a rilevare valori che non siano effettivamente
    binarizzati come previsto.
    """
    value = float(postprocessed_gripper)

    if not np.isfinite(value):
        raise ValueError(
            f"Invalid gripper value: {value}"
        )

    if tolerance < 0.0:
        raise ValueError(
            f"tolerance must be >= 0, got {tolerance}"
        )

    if abs(value - 1.0) <= tolerance:
        return GRIPPER_OPEN

    if abs(value + 1.0) <= tolerance:
        return GRIPPER_CLOSED

    raise ValueError(
        "Unexpected VLA-JEPA postprocessed gripper value. "
        f"Expected approximately -1 or +1, got {value}."
    )


def gripper_binary_to_moveit(
    gripper_state: int | bool,
    open_position: float = 0.0,
    closed_position: float = 255.0,
) -> float:
    """
    Converte la convenzione comune:

        0 = OPEN
        1 = CLOSED

    nel comando utilizzato dal Robotiq/MoveIt.
    """
    state = int(gripper_state)

    if state not in (GRIPPER_OPEN, GRIPPER_CLOSED):
        raise ValueError(
            "gripper_state must be 0 (open) or 1 (closed), "
            f"got {gripper_state}"
        )

    return float(
        closed_position
        if state == GRIPPER_CLOSED
        else open_position
    )


# =============================================================================
# DELTA POSE -> TARGET ASSOLUTO
# =============================================================================


def delta_action_to_absolute_target(
    postprocessed_action: torch.Tensor | np.ndarray,
    reference_position: np.ndarray,
    reference_quaternion_xyzw: np.ndarray,
    scale_factor: float = DATASET_ACTION_SCALE,
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    Converte una singola action VLA-JEPA nel target assoluto UR5e.

    VLAJEPAPolicy.select_action() restituisce una action alla volta.
    Non è quindi necessario convertire manualmente un intero action chunk.

    Action:
        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    Dopo il recupero della scala fisica:

        p_new = p_current + delta_p

        R_delta = Rotation.from_euler("xyz", delta_rpy)

        R_new = R_delta @ R_current

    La moltiplicazione a sinistra replica la convenzione utilizzata
    nella costruzione delle action UR5e: il delta di rotazione è espresso
    rispetto agli assi del frame base_link.

    Args:
        postprocessed_action:
            Action [7] già passata attraverso il postprocessor LeRobot.

        reference_position:
            Posizione corrente reale dell'end-effector:
            [x, y, z].

        reference_quaternion_xyzw:
            Quaternione corrente ROS/SciPy:
            [qx, qy, qz, qw].

        scale_factor:
            Fattore necessario per recuperare i delta fisici dalle
            label scalate del dataset.

    Returns:
        target_position:
            np.float32, shape (3,)

        target_quaternion_xyzw:
            np.float32, shape (4,)

        gripper_state:
            int:
                0 = open
                1 = closed
    """
    action = validate_vla_jepa_action(
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

    quaternion_norm = np.linalg.norm(
        reference_quaternion_xyzw
    )

    if quaternion_norm < 1e-8:
        raise ValueError(
            "reference_quaternion_xyzw has near-zero norm."
        )

    reference_quaternion_xyzw /= quaternion_norm

    # ------------------------------------------------------------------
    # Traslazione
    # ------------------------------------------------------------------

    delta_position = physical_delta[:3].astype(
        np.float64
    )

    target_position = (
        reference_position + delta_position
    )

    # ------------------------------------------------------------------
    # Rotazione
    # ------------------------------------------------------------------

    delta_rpy = physical_delta[3:6].astype(
        np.float64
    )

    current_rotation = Rotation.from_quat(
        reference_quaternion_xyzw
    ).as_matrix()

    delta_rotation = Rotation.from_euler(
        "xyz",
        delta_rpy,
        degrees=False,
    ).as_matrix()

    # Convenzione dataset:
    #
    #     R_new = R_delta @ R_current
    #
    target_rotation = (
        delta_rotation @ current_rotation
    )

    target_quaternion_xyzw = Rotation.from_matrix(
        target_rotation
    ).as_quat()

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    gripper_state = decode_gripper_binary(
        action[6]
    )

    return (
        target_position.astype(np.float32),
        target_quaternion_xyzw.astype(np.float32),
        gripper_state,
    )