from __future__ import annotations

from collections.abc import Callable
from typing import Literal

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from scipy.spatial.transform import Rotation


FRONT_CROP_MARGINS = (0, 10, 130, 100)  # top, bottom, left, right --> valori presi dal codice che crea tfrecord
DATASET_IMAGE_SIZE = (224, 224)  # width, height
PREPROCESS_VIDEO_SIZE = (320, 240)  # width, height
MODEL_IMAGE_SIZE = (640, 480)  # width, height
ROTATION_EPS = 1e-8


# Viene eseguita durante il preprocessing visivo, dopo il crop usato per creare i
# TFRecord: conserva le proporzioni dell'immagine e aggiunge padding nero fino a 4:3.
def resize_with_padding(
    image: np.ndarray,
    output_size: tuple[int, int] = PREPROCESS_VIDEO_SIZE,
) -> np.ndarray:
    output_width, output_height = output_size
    height, width = image.shape[:2]
    scale = min(output_width / width, output_height / height)

    resized_width = round(width * scale)
    resized_height = round(height * scale)
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )

    canvas = np.zeros((output_height, output_width, 3), dtype=np.uint8)
    top = (output_height - resized_height) // 2
    left = (output_width - resized_width) // 2
    canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas


# Viene eseguita dal controller per ogni nuovo frame ROS, prima di inserirlo nella
# storia temporale: replica crop, resize, padding e normalizzazione usati nel training.
def process_rgb_image(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"] = "rgb",
    crop_margins: tuple[int, int, int, int] = FRONT_CROP_MARGINS,
) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError(f"Expected an HWC uint8 image with 3 channels, got {image.shape} {image.dtype}")
    if input_color_order not in {"rgb", "bgr"}:
        raise ValueError(f"Unsupported input color order: {input_color_order}")

    if input_color_order == "bgr":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    top, bottom, left, right = crop_margins
    height, width = image.shape[:2]
    if min(top, bottom, left, right) < 0 or top + bottom >= height or left + right >= width:
        raise ValueError(f"Invalid crop margins {crop_margins} for image shape {image.shape}")

    cropped = image[top : height - bottom, left : width - right]
    dataset_image = cv2.resize(cropped, DATASET_IMAGE_SIZE, interpolation=cv2.INTER_LINEAR)
    padded = resize_with_padding(dataset_image)

    tensor = torch.from_numpy(np.ascontiguousarray(padded)).permute(2, 0, 1).float()
    tensor = F.interpolate(
        tensor.unsqueeze(0),
        size=(MODEL_IMAGE_SIZE[1], MODEL_IMAGE_SIZE[0]),
        mode="bilinear",
        align_corners=False,
        antialias=True,
    ).squeeze(0)
    tensor = 2.0 * (tensor / 255.0 - 0.5)
    return np.ascontiguousarray(tensor.numpy()[:, None], dtype=np.float32)


# Viene eseguita mentre il controller prepara lo stato per l'Action Head: converte
# il quaternion ROS XYZW assoluto in una matrice di rotazione nel frame base_link.
def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError(f"Expected a finite XYZW quaternion with shape (4,), got {quaternion}")

    norm = np.linalg.norm(quaternion)
    if norm < ROTATION_EPS:
        raise ValueError("Cannot convert a zero-norm quaternion")
    return Rotation.from_quat(quaternion / norm).as_matrix()

# Viene eseguita durante la preparazione dello stato 7D: converte
# il quaternion ROS XYZW assoluto direttamente in RPY xyz, nella stessa
# convenzione usata per eef_rot_lowdim nel nuovo dataset.
def quaternion_to_rpy(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError(
            f"Expected a finite XYZW quaternion with shape (4,), got {quaternion}"
        )

    norm = np.linalg.norm(quaternion)
    if norm < ROTATION_EPS:
        raise ValueError("Cannot convert a zero-norm quaternion")

    return Rotation.from_quat(
        quaternion / norm
    ).as_euler("xyz").astype(np.float32)


# Viene eseguita nella preparazione dello stato 10D: mantiene le prime due righe
# della matrice assoluta, nella stessa convenzione usata dagli YAML di training.
# def rotation_matrix_to_6d(rotation_matrix: np.ndarray) -> np.ndarray:
#     rotation_matrix = np.asarray(rotation_matrix, dtype=np.float64)
#     if rotation_matrix.shape != (3, 3) or not np.all(np.isfinite(rotation_matrix)):
#         raise ValueError(f"Expected a finite rotation matrix with shape (3, 3), got {rotation_matrix.shape}")
#     return rotation_matrix[:2].reshape(6).astype(np.float32)


# Viene eseguita dopo aver ricevuto posa e gripper dal nodo ROS: costruisce il
# vettore propriocettivo (10,) che il controller porterà alla shape (1, 1, 10).
# Sostituisce le trasformazioni configurate nei file YAML del training.
def build_lowdim_state(
    eef_position: np.ndarray,
    eef_quaternion: np.ndarray,
    gripper_state: float,
) -> np.ndarray:
    eef_position = np.asarray(eef_position, dtype=np.float64)
    if eef_position.shape != (3,) or not np.all(np.isfinite(eef_position)):
        raise ValueError(f"Expected a finite EEF position with shape (3,), got {eef_position}")
    if not np.isfinite(gripper_state) or not 0.0 <= gripper_state <= 1.0:
        raise ValueError(f"Expected a gripper state in [0, 1], got {gripper_state}")

    # rotation_matrix = quaternion_to_rotation_matrix(eef_quaternion)
    # rotation_6d = rotation_matrix_to_6d(rotation_matrix)
    # return np.concatenate((eef_position, rotation_6d, [gripper_state])).astype(np.float32)
    
    eef_rpy = quaternion_to_rpy(eef_quaternion)
    return np.concatenate(
        (
            eef_position,
            eef_rpy,
            [gripper_state],
        )
    ).astype(np.float32)

# Viene eseguita sul blocco rotazionale 6D prodotto dall'Action Head: applica
# Gram-Schmidt e ricostruisce una matrice relativa ortonormale con determinante +1.
# def rotation_6d_to_matrix(rotation_6d: np.ndarray) -> np.ndarray:
#     rotation_6d = np.asarray(rotation_6d, dtype=np.float64)
#     if rotation_6d.shape != (6,) or not np.all(np.isfinite(rotation_6d)):
#         raise ValueError(f"Expected a finite 6D rotation with shape (6,), got {rotation_6d}")

#     first = rotation_6d[:3]
#     second = rotation_6d[3:]
#     first_norm = np.linalg.norm(first)
#     if first_norm < ROTATION_EPS:
#         raise ValueError("The first 6D rotation vector has zero norm")

#     first = first / first_norm
#     second = second - np.dot(second, first) * first
#     second_norm = np.linalg.norm(second)
#     if second_norm < ROTATION_EPS:
#         raise ValueError("The two 6D rotation vectors are collinear")

#     second = second / second_norm
#     third = np.cross(first, second)
#     return np.stack((first, second, third), axis=0)

# Viene eseguita sul blocco rotazionale RPY prodotto dal nuovo Action Head 7D:
# interpreta [droll, dpitch, dyaw] come rotazione relativa nel frame base_link
# e la converte in matrice per poterla comporre con la posa corrente.
def delta_rpy_to_rotation_matrix(delta_rpy: np.ndarray) -> np.ndarray:
    delta_rpy = np.asarray(delta_rpy, dtype=np.float64)

    if delta_rpy.shape != (3,) or not np.all(np.isfinite(delta_rpy)):
        raise ValueError(
            f"Expected finite delta RPY with shape (3,), got {delta_rpy}"
        )

    return Rotation.from_euler(
        "xyz",
        delta_rpy,
    ).as_matrix()


# Viene eseguita dopo aver composto rotazione relativa e stato corrente: converte
# il target assoluto 3x3 nel quaternion XYZW richiesto dal messaggio Pose di MoveIt.
def rotation_matrix_to_quaternion(
    rotation_matrix: np.ndarray,
    reference_quaternion: np.ndarray | None = None,
) -> np.ndarray:
    rotation_matrix = np.asarray(rotation_matrix, dtype=np.float64)
    if rotation_matrix.shape != (3, 3) or not np.all(np.isfinite(rotation_matrix)):
        raise ValueError(f"Expected a finite rotation matrix with shape (3, 3), got {rotation_matrix.shape}")

    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
    if reference_quaternion is not None:
        reference_quaternion = np.asarray(reference_quaternion, dtype=np.float64)
        if reference_quaternion.shape != (4,):
            raise ValueError(f"Expected reference quaternion shape (4,), got {reference_quaternion.shape}")
        if np.dot(quaternion, reference_quaternion) < 0:
            quaternion = -quaternion
    return quaternion.astype(np.float32)


# Viene eseguita per ogni action mentre il controller costruisce il buffer assoluto:
# stabilizza l'uscita continua della rete e la converte nella scala del Robotiq.
def convert_gripper(
    model_value: float,
    currently_closed: bool,
    close_threshold: float = 0.9,
    open_threshold: float = 0.7,
    open_position: float = 0.0,
    closed_position: float = 255.0,
) -> tuple[float, bool]:
    if not np.isfinite(model_value):
        raise ValueError(f"Invalid gripper prediction: {model_value}")
    if not 0.0 <= open_threshold <= close_threshold <= 1.0:
        raise ValueError("Expected 0 <= open_threshold <= close_threshold <= 1")

    is_closed = model_value >= open_threshold if currently_closed else model_value > close_threshold
    command = closed_position if is_closed else open_position
    return float(command), bool(is_closed)


# Viene eseguita dopo una query Mimic Video e prima di riempire l'action buffer:
# integra le delta nel frame base_link e restituisce pose assolute 8D per il nodo ROS.
def action_chunk_to_absolute_poses(
    action_chunk: np.ndarray,
    current_position: np.ndarray,
    current_quaternion: np.ndarray,
    gripper_closed: bool,
    close_threshold: float = 0.9,
    open_threshold: float = 0.7,
    open_position: float = 0.0,
    closed_position: float = 255.0,
    diagnostics_callback: Callable[[dict[str, object]], None] | None = None,
) -> tuple[np.ndarray, bool]:
    action_chunk = np.asarray(action_chunk, dtype=np.float64)
    if action_chunk.ndim == 3 and action_chunk.shape[0] == 1:
        action_chunk = action_chunk[0]
    # if action_chunk.ndim != 2 or action_chunk.shape[1] != 10 or not np.all(np.isfinite(action_chunk)):
    #     raise ValueError(f"Expected a finite action chunk with shape (H, 10), got {action_chunk.shape}")

    if (
        action_chunk.ndim != 2
        or action_chunk.shape[1] != 7
        or not np.all(np.isfinite(action_chunk))
    ):
        raise ValueError(
            f"Expected a finite action chunk with shape (H, 7), got {action_chunk.shape}"
        )


    position = np.asarray(current_position, dtype=np.float64).copy()
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError(f"Expected a finite current position with shape (3,), got {position}")
    quaternion = np.asarray(current_quaternion, dtype=np.float64).copy()
    rotation = quaternion_to_rotation_matrix(quaternion)

    absolute_poses = np.empty((len(action_chunk), 8), dtype=np.float32)
    for index, action in enumerate(action_chunk):
        previous_position = position.copy()
        previous_quaternion = quaternion.copy()
        previous_rotation = rotation.copy()
        was_closed = gripper_closed

        position = position + action[:3]
        # raw_first_row = action[3:6]
        # raw_second_row = action[6:9]
        # delta_rotation = rotation_6d_to_matrix(action[3:9])
        delta_rpy = action[3:6]
        delta_rotation = delta_rpy_to_rotation_matrix(delta_rpy)
        rotation = delta_rotation @ rotation
        quaternion = rotation_matrix_to_quaternion(rotation, reference_quaternion=quaternion)
        gripper_command, gripper_closed = convert_gripper(
            #action[9],
            action[6],
            gripper_closed,
            close_threshold=close_threshold,
            open_threshold=open_threshold,
            open_position=open_position,
            closed_position=closed_position,
        )
        absolute_poses[index] = np.concatenate((position, quaternion, [gripper_command]))

        if diagnostics_callback is not None:
            diagnostics_callback(
                {
                    "index": index,
                    "raw_action": action.copy(),
                    "previous_position": previous_position,
                    "target_position": position.copy(),
                    "previous_quaternion": previous_quaternion,
                    "target_quaternion": quaternion.copy(),
                    "previous_rotation": previous_rotation,
                    "delta_rotation": delta_rotation.copy(),
                    "target_rotation": rotation.copy(),
                    # "raw_rotation_row_norms": np.array(
                    #     [np.linalg.norm(raw_first_row), np.linalg.norm(raw_second_row)],
                    #     dtype=np.float64,
                    # ),
                    # "raw_rotation_row_dot": float(np.dot(raw_first_row, raw_second_row)),
                    "raw_delta_rpy": delta_rpy.copy(),

                    "raw_delta_rpy_deg": np.degrees(
                        delta_rpy
                    ).astype(np.float64),
                    "delta_rotation_determinant": float(np.linalg.det(delta_rotation)),
                    "delta_rotation_orthogonality_error": float(
                        np.linalg.norm(delta_rotation @ delta_rotation.T - np.eye(3))
                    ),
                    "delta_euler_xyz_deg": Rotation.from_matrix(delta_rotation).as_euler(
                        "xyz", degrees=True
                    ),
                    "delta_angle_deg": float(np.degrees(Rotation.from_matrix(delta_rotation).magnitude())),
                    "target_euler_xyz_deg": Rotation.from_matrix(rotation).as_euler("xyz", degrees=True),
                    "gripper_was_closed": was_closed,
                    "gripper_is_closed": gripper_closed,
                    "gripper_command": gripper_command,
                }
            )

    return absolute_poses, gripper_closed
