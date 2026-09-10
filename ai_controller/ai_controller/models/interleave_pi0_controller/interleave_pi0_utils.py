from __future__ import annotations

from pathlib import Path
from typing import Literal

import cv2
import numpy as np
import torch
from PIL import Image
from scipy.spatial.transform import Rotation


# =============================================================================
# CONFIGURAZIONE UR5e
# =============================================================================

IMAGE_SIZE = 224

# Convenzione:
#   (top, bottom, left, right)
#
# Sono gli stessi margini usati durante la costruzione del dataset originale
# UR5e prima del resize della camera frontale a 224x224.
FRONT_CROP_MARGINS = (0, 10, 140, 90) # top, bottom, left, right

PROPRIO_DIM = 7
ACTION_DIM = 7

# Nel training Interleave-Pi0:
#
#   proprio = [x, y, z, roll, pitch, yaw, gripper]
#
# Tutte le 7 componenti vengono normalizzate con BOUNDS.
PROPRIO_NORMALIZATION_MASK = np.array(
    [True, True, True, True, True, True, True],
    dtype=bool,
)

# action = [dx, dy, dz, droll, dpitch, dyaw, gripper]
#
# Solo le prime sei componenti vengono normalizzate.
# Il comando del gripper resta nel dominio originale 0/1.
ACTION_NORMALIZATION_MASK = np.array(
    [True, True, True, True, True, True, False],
    dtype=bool,
)


# =============================================================================
# UTILITY COMUNI
# =============================================================================


def _as_float_array(
    value,
    expected_shape: tuple[int, ...],
    name: str,
) -> np.ndarray:
    """
    Converte un valore numerico in NumPy float32 e ne verifica la shape.

    È una funzione interna usata dalle utility di preprocessing e
    postprocessing per intercettare immediatamente input incompatibili.
    """
    array = np.asarray(value, dtype=np.float32)

    if array.shape != expected_shape:
        raise ValueError(
            f"{name} must have shape {expected_shape}, "
            f"got {array.shape}"
        )

    return array


def _validate_uint8_rgb_image(
    image: np.ndarray,
    name: str,
) -> np.ndarray:
    """
    Verifica che un'immagine sia HWC, a tre canali e uint8.

    Non applica alcuna trasformazione.
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
# PRE_PROCESS
# =============================================================================
#
# Le funzioni di questa sezione vengono richiamate da:
#
#     InterleavePi0Controller.pre_process()
#
# prima di ogni query al modello.
#
# Il loro compito è trasformare:
#
#     camera live + stato robot + comando selezionato
#
# negli input numerici che InterleavePi0Policy si aspetta.
#
# NON eseguono il preprocessing interno PaliGemma/SigLIP:
# quello rimane responsabilità di InterleavedVLAProcessor.
# =============================================================================


def process_front_image(
    image: np.ndarray,
    input_color_order: Literal["rgb", "bgr"] = "rgb",
    crop_margins: tuple[int, int, int, int] = FRONT_CROP_MARGINS,
) -> torch.Tensor:
    """
    Prepara il frame corrente della camera frontale.

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata dentro `pre_process()` a ogni nuova query al modello,
    perché la observation image cambia continuamente durante il rollout.

    COSA REPLICA
    ------------
    Replica il preprocessing utilizzato durante la costruzione del dataset
    UR5e originale:

        frame camera originale
            -> eventuale BGR -> RGB
            -> crop globale della scena
            -> resize 224x224 con cv2.INTER_LINEAR
            -> HWC -> CHW

    Non viene applicata l'espansione del 20%:
    quella riguardava esclusivamente la bounding box usata per costruire
    l'instruction image dell'oggetto.

    Non viene applicata la normalizzazione dei pixel:
    sarà eseguita successivamente da InterleavedVLAProcessor.

    Args:
        image:
            Immagine HWC uint8 proveniente dalla camera live.

        input_color_order:
            "rgb" se il controller riceve già RGB.
            "bgr" se il frame proviene, ad esempio, da OpenCV/CvBridge
            in encoding BGR.

        crop_margins:
            (top, bottom, left, right).

    Returns:
        torch.uint8 con shape (3, 224, 224).
    """
    image = _validate_uint8_rgb_image(
        image,
        name="front_image",
    )

    if input_color_order == "bgr":
        image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2RGB,
        )
    elif input_color_order != "rgb":
        raise ValueError(
            f"Unsupported input_color_order: {input_color_order}"
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

    # Stesso ordine del dataset:
    # prima crop sul frame originale...
    cropped = image[
        top : height - bottom,
        left : width - right,
    ]

    # ...poi resize diretto a 224x224.
    resized = cv2.resize(
        cropped,
        (IMAGE_SIZE, IMAGE_SIZE),
        interpolation=cv2.INTER_LINEAR,
    )

    # InterleaveVLAProcessor riceve le immagini in formato CHW.
    tensor = torch.from_numpy(
        np.ascontiguousarray(resized)
    ).permute(2, 0, 1)

    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    if tuple(tensor.shape) != expected_shape:
        raise RuntimeError(
            f"Unexpected front image shape: {tuple(tensor.shape)}"
        )

    return tensor


def load_instruction_image(
    image_path: str | Path,
) -> torch.Tensor:
    """
    Carica una delle quattro instruction image statiche:

        green_box.png
        yellow_box.png
        blue_box.png
        red_box.png

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata quando il controller riceve/seleziona un nuovo task,
    tipicamente dentro `load_command()` oppure una sola volta all'inizio
    del rollout.

    Non è necessario ricaricare questa immagine a ogni query:
    il riferimento visivo resta costante per tutta l'esecuzione del task.

    PREPROCESSING
    -------------
    I PNG sono stati estratti direttamente dal dataset Interleave usato
    per il training. Sono quindi già:

        - RGB;
        - crop dell'oggetto;
        - bounding box già espansa del 20%;
        - resize a 224x224;
        - uint8.

    Per questo motivo questa funzione NON applica:
        - crop;
        - espansione della bbox;
        - resize.

    Effettua soltanto il caricamento, la verifica e HWC -> CHW.

    Returns:
        torch.uint8 con shape (3, 224, 224).
    """
    image_path = Path(image_path).expanduser().resolve()

    if not image_path.is_file():
        raise FileNotFoundError(image_path)

    # Pillow restituisce esplicitamente RGB indipendentemente dal formato
    # interno del PNG.
    with Image.open(image_path) as image:
        image_rgb = np.asarray(
            image.convert("RGB"),
            dtype=np.uint8,
        )

    expected_shape = (
        IMAGE_SIZE,
        IMAGE_SIZE,
        3,
    )

    if image_rgb.shape != expected_shape:
        raise ValueError(
            f"Instruction image {image_path} must already have shape "
            f"{expected_shape}, got {image_rgb.shape}"
        )

    return torch.from_numpy(
        np.ascontiguousarray(image_rgb)
    ).permute(2, 0, 1)


def stack_interleaved_images(
    front_image: torch.Tensor,
    instruction_image: torch.Tensor,
) -> torch.Tensor:
    """
    Costruisce il tensor delle immagini da passare a
    InterleavePi0Policy.process_interleaved_inputs().

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata dentro `pre_process()` dopo:

        process_front_image(...)
        load_instruction_image(...)

    L'ordine deve essere identico al training:

        images[0] = observation image corrente
        images[1] = instruction image del target

    Returns:
        torch.uint8 con shape (2, 3, 224, 224).
    """
    expected_shape = (
        3,
        IMAGE_SIZE,
        IMAGE_SIZE,
    )

    for name, image in (
        ("front_image", front_image),
        ("instruction_image", instruction_image),
    ):
        if tuple(image.shape) != expected_shape:
            raise ValueError(
                f"{name} must have shape {expected_shape}, "
                f"got {tuple(image.shape)}"
            )

        if image.dtype != torch.uint8:
            raise TypeError(
                f"{name} must have dtype torch.uint8, "
                f"got {image.dtype}"
            )

    return torch.stack(
        (
            front_image,
            instruction_image,
        ),
        dim=0,
    )


def prepare_interleaved_prompt(
    prompt: str,
) -> str:
    """
    Converte il prompt salvato nello YAML nel formato atteso dal processor.

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata quando viene caricato un nuovo task, non necessariamente
    a ogni query.

    Nel dataset il prompt è memorizzato nella forma:

        "Pick the <image> and place it into the second bin"

    mentre immediatamente prima di InterleavedVLAProcessor il training
    sostituiva:

        <image> -> <image_placeholder>

    Questa funzione replica esattamente quella trasformazione.
    """
    if not isinstance(prompt, str):
        raise TypeError(
            f"prompt must be a string, got {type(prompt)}"
        )

    num_placeholders = prompt.count("<image>")

    if num_placeholders != 1:
        raise ValueError(
            "The UR5e prompt must contain exactly one "
            f"'<image>' placeholder, got {num_placeholders}: {prompt!r}"
        )

    return prompt.replace(
        "<image>",
        "<image_placeholder>",
    )


def build_proprio(
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
    gripper_closed: float | bool,
) -> np.ndarray:
    """
    Costruisce lo stato propriocettivo UR5e nel formato usato nel training.

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata dentro `pre_process()` a ogni query al modello,
    utilizzando la pose corrente reale del robot.

    Input:
        position:
            [x, y, z] dell'end-effector nel frame base_link.

        quaternion_xyzw:
            orientamento corrente nel formato ROS/SciPy:
            [qx, qy, qz, qw].

        gripper_closed:
            stato binario del gripper:
                0 = aperto
                1 = chiuso

    Output:
        [x, y, z, roll, pitch, yaw, gripper]

    Gli angoli sono Euler XYZ, cioè la stessa convenzione usata nel dataset.
    """
    position = _as_float_array(
        position,
        expected_shape=(3,),
        name="position",
    )

    quaternion_xyzw = _as_float_array(
        quaternion_xyzw,
        expected_shape=(4,),
        name="quaternion_xyzw",
    )

    quaternion_norm = np.linalg.norm(quaternion_xyzw)

    if quaternion_norm < 1e-8:
        raise ValueError(
            "quaternion_xyzw has near-zero norm."
        )

    # Normalizziamo il quaternione per tollerare piccoli errori numerici
    # provenienti dalla lettura ROS.
    quaternion_xyzw = (
        quaternion_xyzw / quaternion_norm
    )

    rpy = Rotation.from_quat(
        quaternion_xyzw
    ).as_euler(
        "xyz",
        degrees=False,
    )

    gripper = np.float32(
        float(gripper_closed) >= 0.5
    )

    proprio = np.concatenate(
        (
            position,
            rpy.astype(np.float32),
            np.array([gripper], dtype=np.float32),
        )
    )

    return proprio.astype(
        np.float32,
        copy=False,
    )


def normalize_bounds(
    values: np.ndarray,
    p01: np.ndarray,
    p99: np.ndarray,
    mask: np.ndarray,
) -> np.ndarray:
    """
    Applica la normalizzazione BOUNDS usata durante il training.

    QUANDO VIENE ESEGUITA
    --------------------
    In `pre_process()` viene usata per normalizzare il proprio corrente
    prima di fornirlo al modello.

    Formula per ogni componente selezionata:

        normalized =
            clip(
                2 * (value - p01) / (p99 - p01) - 1,
                -1,
                1
            )

    Le componenti con mask=False rimangono invariate.

    IMPORTANTE
    ----------
    `p01` e `p99` devono essere ESATTAMENTE quelli calcolati sul train set
    usato per il fine-tuning.
    """
    values = np.asarray(
        values,
        dtype=np.float32,
    )

    p01 = np.asarray(
        p01,
        dtype=np.float32,
    )

    p99 = np.asarray(
        p99,
        dtype=np.float32,
    )

    mask = np.asarray(
        mask,
        dtype=bool,
    )

    if (
        values.shape != p01.shape
        or values.shape != p99.shape
        or values.shape != mask.shape
    ):
        raise ValueError(
            "values, p01, p99 and mask must have the same shape: "
            f"values={values.shape}, "
            f"p01={p01.shape}, "
            f"p99={p99.shape}, "
            f"mask={mask.shape}"
        )

    denominator = p99 - p01

    if np.any(
        np.abs(denominator[mask]) < 1e-8
    ):
        raise ValueError(
            "At least one normalized dimension has p01 == p99."
        )

    normalized = values.copy()

    normalized[mask] = (
        2.0
        * (values[mask] - p01[mask])
        / denominator[mask]
        - 1.0
    )

    normalized[mask] = np.clip(
        normalized[mask],
        -1.0,
        1.0,
    )

    return normalized.astype(
        np.float32,
        copy=False,
    )


def prepare_proprio_tensor(
    proprio: np.ndarray,
    proprio_p01: np.ndarray,
    proprio_p99: np.ndarray,
) -> torch.Tensor:
    """
    Normalizza il proprio e aggiunge batch e dimensione temporale.

    QUANDO VIENE ESEGUITA
    --------------------
    È l'ultimo passaggio sullo stato robot dentro `pre_process()`.

    Input:
        proprio.shape == (7,)

    Output:
        tensor.shape == (1, 1, 7)

    Le dimensioni rappresentano:

        batch_size = 1
        cond_steps = 1
        proprio_dim = 7
    """
    proprio = _as_float_array(
        proprio,
        expected_shape=(PROPRIO_DIM,),
        name="proprio",
    )

    normalized = normalize_bounds(
        values=proprio,
        p01=np.asarray(proprio_p01, dtype=np.float32),
        p99=np.asarray(proprio_p99, dtype=np.float32),
        mask=PROPRIO_NORMALIZATION_MASK,
    )

    return torch.from_numpy(
        normalized
    ).view(
        1,
        1,
        PROPRIO_DIM,
    )


# =============================================================================
# POST_PROCESS
# =============================================================================
#
# Le funzioni di questa sezione vengono richiamate da:
#
#     InterleavePi0Controller.post_process()
#
# dopo:
#
#     InterleavePi0Policy.predict(...)
#
# Il modello restituisce un chunk normalizzato:
#
#     (1, 4, 7)
#
# Queste utility riportano le action nello spazio fisico UR5e e trasformano
# le delta pose nelle pose assolute che il controller potrà poi inviare a
# MoveIt.
# =============================================================================


def denormalize_bounds(
    values: np.ndarray,
    p01: np.ndarray,
    p99: np.ndarray,
    mask: np.ndarray,
) -> np.ndarray:
    """
    Inverte la normalizzazione BOUNDS.

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata nel `post_process()` immediatamente dopo la predizione
    del modello.

    Formula inversa:

        value =
            (normalized + 1) / 2 * (p99 - p01) + p01

    Le componenti mask=False rimangono invariate.

    Per le action UR5e:
        - dx, dy, dz, droll, dpitch, dyaw vengono denormalizzati;
        - il gripper non viene modificato.
    """
    values = np.asarray(
        values,
        dtype=np.float32,
    )

    p01 = np.asarray(
        p01,
        dtype=np.float32,
    )

    p99 = np.asarray(
        p99,
        dtype=np.float32,
    )

    mask = np.asarray(
        mask,
        dtype=bool,
    )

    if (
        values.shape != p01.shape
        or values.shape != p99.shape
        or values.shape != mask.shape
    ):
        raise ValueError(
            "values, p01, p99 and mask must have the same shape."
        )

    denormalized = values.copy()

    denormalized[mask] = (
        (values[mask] + 1.0)
        * 0.5
        * (p99[mask] - p01[mask])
        + p01[mask]
    )

    return denormalized.astype(
        np.float32,
        copy=False,
    )


def denormalize_action_chunk(
    action_chunk: torch.Tensor | np.ndarray,
    action_p01: np.ndarray,
    action_p99: np.ndarray,
) -> np.ndarray:
    """
    Riporta l'intero action chunk nello spazio fisico UR5e.

    QUANDO VIENE ESEGUITA
    --------------------
    È la prima operazione di `post_process()`.

    Il modello restituisce:
        (1, 4, 7)

    Questa funzione restituisce:
        (4, 7)

    con:
        [dx, dy, dz, droll, dpitch, dyaw, gripper]

    nelle unità originali del dataset:
        - metri;
        - radianti;
        - gripper 0/1 circa.
    """
    if isinstance(action_chunk, torch.Tensor):
        action_chunk = (
            action_chunk
            .detach()
            .to("cpu")
            .float()
            .numpy()
        )
    else:
        action_chunk = np.asarray(
            action_chunk,
            dtype=np.float32,
        )

    if action_chunk.shape == (1, 4, ACTION_DIM):
        action_chunk = action_chunk[0]

    expected_shape = (
        4,
        ACTION_DIM,
    )

    if action_chunk.shape != expected_shape:
        raise ValueError(
            f"action_chunk must have shape (1, 4, 7) or {expected_shape}, "
            f"got {action_chunk.shape}"
        )

    action_p01 = _as_float_array(
        action_p01,
        expected_shape=(ACTION_DIM,),
        name="action_p01",
    )

    action_p99 = _as_float_array(
        action_p99,
        expected_shape=(ACTION_DIM,),
        name="action_p99",
    )

    result = np.empty_like(
        action_chunk,
        dtype=np.float32,
    )

    for i, action in enumerate(action_chunk):
        result[i] = denormalize_bounds(
            values=action,
            p01=action_p01,
            p99=action_p99,
            mask=ACTION_NORMALIZATION_MASK,
        )

    return result


def convert_gripper(
    model_value: float,
    currently_closed: bool,
    close_threshold: float = 0.9,
    open_threshold: float = 0.7,
    open_position: float = 0.0,
    closed_position: float = 255.0,
) -> tuple[float, bool]:
    if not np.isfinite(model_value):
        raise ValueError(
            f"Invalid gripper prediction: {model_value}"
        )

    if not 0.0 <= open_threshold <= close_threshold <= 1.0:
        raise ValueError(
            "Expected 0 <= open_threshold <= close_threshold <= 1"
        )

    is_closed = (
        model_value >= open_threshold
        if currently_closed
        else model_value > close_threshold
    )

    command = (
        closed_position
        if is_closed
        else open_position
    )

    return float(command), bool(is_closed)


def delta_action_chunk_to_absolute_targets(
    action_chunk: np.ndarray,
    reference_position: np.ndarray,
    reference_quaternion_xyzw: np.ndarray,
    gripper_closed: bool,
    close_threshold: float = 0.9,
    open_threshold: float = 0.7,
    open_position: float = 0.0,
    closed_position: float = 255.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Integra sequenzialmente le delta-action previste dal modello.

    QUANDO VIENE ESEGUITA
    --------------------
    Viene chiamata in `post_process()` dopo
    `denormalize_action_chunk()`.

    Serve perché Interleave-Pi0 produce delta pose, mentre MoveIt lavora
    con target di pose assoluti.

    CONVENZIONE DEL DATASET
    -----------------------
    Per ogni action:

        p_next = p_current + delta_p

        R_delta = Rotation.from_euler(
            "xyz",
            delta_rpy
        )

        R_next = R_delta @ R_current

    La rotazione relativa viene quindi applicata rispetto agli assi
    del frame base_link, esattamente come nella costruzione delle action
    del dataset.

    IMPORTANTE
    ----------
    Le quattro action vengono integrate SEQUENZIALMENTE.

    action[0] è relativa alla pose corrente;
    action[1] è relativa al target prodotto da action[0];
    action[2] è relativa al target prodotto da action[1];
    ecc.

    Returns:
        positions:
            shape (4, 3)

        quaternions_xyzw:
            shape (4, 4)

        gripper_commands:
            shape (4,)
    """
    action_chunk = np.asarray(
        action_chunk,
        dtype=np.float32,
    )

    if (
        action_chunk.ndim != 2
        or action_chunk.shape[1] != ACTION_DIM
    ):
        raise ValueError(
            "action_chunk must have shape (H, 7), "
            f"got {action_chunk.shape}"
        )

    current_position = _as_float_array(
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

    current_rotation = Rotation.from_quat(
        reference_quaternion_xyzw
    ).as_matrix()

    horizon = action_chunk.shape[0]

    positions = np.empty(
        (horizon, 3),
        dtype=np.float64,
    )

    quaternions_xyzw = np.empty(
        (horizon, 4),
        dtype=np.float64,
    )

    gripper_commands = np.empty(
        (horizon,),
        dtype=np.float32,
    )

    current_gripper_closed = bool(gripper_closed)

    for i, action in enumerate(action_chunk):

        # -------------------------------------------------------------
        # Traslazione
        # -------------------------------------------------------------
        delta_position = action[:3].astype(
            np.float64
        )

        current_position = (
            current_position + delta_position
        )

        # -------------------------------------------------------------
        # Rotazione
        # -------------------------------------------------------------
        delta_rpy = action[3:6].astype(
            np.float64
        )

        delta_rotation = Rotation.from_euler(
            "xyz",
            delta_rpy,
            degrees=False,
        ).as_matrix()

        # Convenzione verificata nel dataset:
        #
        #     R_next = R_delta @ R_current
        #
        current_rotation = (
            delta_rotation @ current_rotation
        )

        current_quaternion = Rotation.from_matrix(
            current_rotation
        ).as_quat()

        # -------------------------------------------------------------
        # Gripper
        # -------------------------------------------------------------
        gripper_command, current_gripper_closed = convert_gripper(
            model_value=action[6],
            currently_closed=current_gripper_closed,
            close_threshold=close_threshold,
            open_threshold=open_threshold,
            open_position=open_position,
            closed_position=closed_position,
        )

        # -------------------------------------------------------------
        # Offset correttivo per MoveIt
        # -------------------------------------------------------------
        output_position = current_position.copy()

        # Se il target è sotto z = -3 cm,
        # trasla il target di +1 cm lungo x.
        if output_position[2] < -0.03:
            output_position[0] += 0.01

        positions[i] = output_position
        quaternions_xyzw[i] = current_quaternion
        gripper_commands[i] = gripper_command

    return (
        positions.astype(np.float32),
        quaternions_xyzw.astype(np.float32),
        gripper_commands,
    )