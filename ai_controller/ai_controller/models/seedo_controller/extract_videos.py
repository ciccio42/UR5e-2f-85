#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import pickle
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

import cv2
import numpy as np


DEFAULT_FPS = 10.0

CAMERAS = {
    "camera_front": (
        "camera_front_image",
        "front_camera_image",
    ),
    "camera_lateral_left": (
        "camera_lateral_left_image",
        "left_camera_image",
    ),
    "camera_lateral_right": (
        "camera_lateral_right_image",
        "right_camera_image",
    ),
    "eye_in_hand": (
        "eye_in_hand_image",
        "camera_gripper_image",
        "gripper_camera_image",
    ),
}

FPS_KEYS = (
    "fps",
    "capture_fps",
    "camera_fps",
    "video_fps",
    "recording_fps",
    "frame_rate",
    "framerate",
)

TIMESTAMP_KEYS = (
    "timestamp",
    "capture_timestamp",
    "frame_timestamp",
    "ros_timestamp",
    "time_stamp",
)


class TrajectoryProxy:
    def __init__(self, *args, **kwargs):
        self._data = []
        self._raw_state = []
        self._config_str = None

    @property
    def T(self) -> int:
        return len(self._data)

    def __len__(self) -> int:
        return self.T


class TrajectoryUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == "Trajectory":
            return TrajectoryProxy
        return super().find_class(module, name)


def load_pickle(path: Path) -> Any:
    with path.open("rb") as stream:
        return TrajectoryUnpickler(stream).load()


def decode_frame(image: Any) -> np.ndarray:
    image = np.asarray(image)

    if image.dtype == np.uint8 and (
        image.ndim == 1
        or (
            image.ndim == 2
            and image.shape[1] == 1
        )
    ):
        encoded = image.reshape(-1)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

        if frame is None:
            raise ValueError("Impossibile decodificare un frame compresso.")

        return frame

    if image.ndim == 3 and image.shape[2] == 3:
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)

        return image

    raise ValueError(
        f"Formato immagine non supportato: "
        f"shape={image.shape}, dtype={image.dtype}"
    )


def get_observation(entry: Any) -> dict:
    if not isinstance(entry, (tuple, list)) or not entry:
        raise ValueError("Formato trajectory entry non supportato.")

    obs = entry[0]

    if not isinstance(obs, dict):
        raise ValueError("L'osservazione della traiettoria non è un dizionario.")

    return obs


def find_camera_key(first_obs: dict, camera_name: str) -> str:
    candidates = CAMERAS[camera_name]

    for key in candidates:
        if key in first_obs:
            return key

    raise KeyError(
        f"Nessuna immagine trovata per {camera_name}. "
        f"Chiavi accettate: {candidates}. "
        f"Chiavi presenti: {sorted(first_obs.keys())}"
    )


def _valid_fps(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(value) or value <= 0.0 or value > 1000.0:
        return None

    return value


def _find_fps_recursive(
    obj: Any,
    *,
    depth: int = 0,
) -> tuple[float, str] | None:
    if depth > 5:
        return None

    if isinstance(obj, dict):
        lowered = {
            str(key).lower(): key
            for key in obj.keys()
        }

        for candidate in FPS_KEYS:
            original_key = lowered.get(candidate)

            if original_key is None:
                continue

            fps = _valid_fps(obj[original_key])

            if fps is not None:
                return fps, f"metadata:{original_key}"

        for key, value in obj.items():
            if key == "traj":
                continue

            result = _find_fps_recursive(
                value,
                depth=depth + 1,
            )

            if result is not None:
                return result

    elif isinstance(obj, (list, tuple)):
        for value in obj[:50]:
            result = _find_fps_recursive(
                value,
                depth=depth + 1,
            )

            if result is not None:
                return result

    return None


def _find_fps_in_config_string(
    config_str: Any,
) -> tuple[float, str] | None:
    if not isinstance(config_str, str):
        return None

    pattern = re.compile(
        r"(?i)\b("
        + "|".join(re.escape(key) for key in FPS_KEYS)
        + r")\b\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)"
    )

    match = pattern.search(config_str)

    if match is None:
        return None

    fps = _valid_fps(match.group(2))

    if fps is None:
        return None

    return fps, f"config_str:{match.group(1)}"


def _timestamp_to_seconds(value: Any) -> float | None:
    if value is None:
        return None

    if isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        value = float(value)
        return value if math.isfinite(value) else None

    if isinstance(value, dict):
        sec = None
        nsec = 0.0

        for key in ("sec", "secs", "seconds"):
            if key in value:
                try:
                    sec = float(value[key])
                except (TypeError, ValueError):
                    pass
                break

        for key in ("nanosec", "nsec", "nanoseconds"):
            if key in value:
                try:
                    nsec = float(value[key])
                except (TypeError, ValueError):
                    pass
                break

        if sec is not None:
            return sec + nsec * 1e-9

    if hasattr(value, "sec"):
        try:
            sec = float(value.sec)
            nsec = float(getattr(value, "nanosec", 0.0))
            return sec + nsec * 1e-9
        except Exception:
            return None

    return None


def _find_timestamp_recursive(
    obj: Any,
    *,
    depth: int = 0,
) -> float | None:
    if depth > 4 or not isinstance(obj, dict):
        return None

    lowered = {
        str(key).lower(): key
        for key in obj.keys()
    }

    for candidate in TIMESTAMP_KEYS:
        original_key = lowered.get(candidate)

        if original_key is None:
            continue

        timestamp = _timestamp_to_seconds(obj[original_key])

        if timestamp is not None:
            return timestamp

    for value in obj.values():
        if isinstance(value, dict):
            timestamp = _find_timestamp_recursive(
                value,
                depth=depth + 1,
            )

            if timestamp is not None:
                return timestamp

    return None


def _estimate_fps_from_timestamps(
    trajectory,
) -> tuple[float, str] | None:
    timestamps = []
    raw_states = getattr(trajectory, "_raw_state", None)

    for index, entry in enumerate(trajectory._data):
        obs = get_observation(entry)
        candidates = [obs]

        if len(entry) >= 4 and isinstance(entry[3], dict):
            candidates.append(entry[3])

        if (
            isinstance(raw_states, list)
            and index < len(raw_states)
            and isinstance(raw_states[index], dict)
        ):
            candidates.append(raw_states[index])

        timestamp = None

        for candidate in candidates:
            timestamp = _find_timestamp_recursive(candidate)

            if timestamp is not None:
                break

        if timestamp is not None:
            timestamps.append(timestamp)

    if len(timestamps) < 2:
        return None

    timestamps = np.asarray(timestamps, dtype=np.float64)
    deltas = np.diff(timestamps)
    deltas = deltas[np.isfinite(deltas) & (deltas > 0.0)]

    if deltas.size == 0:
        return None

    median_dt = float(np.median(deltas))
    fps = _valid_fps(1.0 / median_dt)

    if fps is None:
        return None

    return fps, "per-frame timestamps (median interval)"


def determine_fps(
    payload: Any,
    trajectory,
) -> tuple[float, str, bool]:
    result = _find_fps_recursive(payload)

    if result is not None:
        fps, source = result
        return fps, source, True

    config_result = _find_fps_in_config_string(
        getattr(trajectory, "_config_str", None)
    )

    if config_result is not None:
        fps, source = config_result
        return fps, source, True

    timestamp_result = _estimate_fps_from_timestamps(trajectory)

    if timestamp_result is not None:
        fps, source = timestamp_result
        return fps, source, True

    return DEFAULT_FPS, "fallback", False


def create_encoder(
    output_path: Path,
    width: int,
    height: int,
    fps: float,
):
    command = [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{width}x{height}",
        "-framerate",
        f"{fps:.10f}",
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_path),
    ]

    return subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
    )


def extract_videos(pkl_path: Path) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("FFmpeg non è installato o non è nel PATH.")

    if not pkl_path.is_file():
        raise FileNotFoundError(f"File non trovato: {pkl_path}")

    if pkl_path.suffix.lower() != ".pkl":
        raise ValueError("Il file di input deve essere un .pkl.")

    payload = load_pickle(pkl_path)

    if not isinstance(payload, dict):
        raise ValueError("Il PKL non contiene un dizionario top-level.")

    trajectory = payload.get("traj")

    if trajectory is None or not hasattr(trajectory, "_data"):
        raise ValueError("Il PKL non contiene una Trajectory valida.")

    if not trajectory._data:
        raise ValueError("La traiettoria è vuota.")

    fps, fps_source, fps_is_from_file = determine_fps(
        payload,
        trajectory,
    )

    print(f"[INFO] Traiettoria: {pkl_path}")
    print(f"[INFO] Step/frame disponibili: {len(trajectory._data)}")

    if fps_is_from_file:
        print(
            f"[INFO] FPS rilevato automaticamente: "
            f"{fps:.6g} ({fps_source})"
        )
    else:
        print(
            "[WARN] Il PKL non contiene FPS né timestamp per-frame: "
            "l'FPS reale originale non è ricostruibile esattamente."
        )
        print(f"[WARN] Uso fallback: {fps:.6g} FPS.")

    first_obs = get_observation(trajectory._data[0])

    camera_keys = {
        camera_name: find_camera_key(
            first_obs,
            camera_name,
        )
        for camera_name in CAMERAS
    }

    print("[INFO] Camere trovate:")
    for camera_name, key in camera_keys.items():
        print(f"       {camera_name}: {key}")

    encoders = {}
    frame_sizes = {}
    frame_counts = {
        camera: 0
        for camera in CAMERAS
    }

    output_paths = {
        camera: (
            pkl_path.parent
            / f"{pkl_path.stem}_{camera}.mp4"
        )
        for camera in CAMERAS
    }

    try:
        for step_index, entry in enumerate(trajectory._data):
            obs = get_observation(entry)

            for camera_name, key in camera_keys.items():
                if key not in obs:
                    raise KeyError(
                        f"Step {step_index}: manca {key!r} "
                        f"per {camera_name}."
                    )

                frame = np.ascontiguousarray(
                    decode_frame(obs[key])
                )

                height, width = frame.shape[:2]

                if camera_name not in encoders:
                    output_path = output_paths[camera_name]

                    encoders[camera_name] = create_encoder(
                        output_path,
                        width,
                        height,
                        fps,
                    )

                    frame_sizes[camera_name] = (
                        width,
                        height,
                    )

                    print(f"[INFO] Creazione: {output_path}")

                if (width, height) != frame_sizes[camera_name]:
                    raise ValueError(
                        f"Risoluzione variabile per {camera_name}: "
                        f"attesa={frame_sizes[camera_name]}, "
                        f"ricevuta={(width, height)} "
                        f"allo step {step_index}."
                    )

                encoder = encoders[camera_name]

                if encoder.stdin is None:
                    raise RuntimeError(
                        f"stdin FFmpeg non disponibile per {camera_name}."
                    )

                encoder.stdin.write(frame.tobytes())
                frame_counts[camera_name] += 1

    except Exception:
        for encoder in encoders.values():
            if encoder.stdin is not None:
                try:
                    encoder.stdin.close()
                except Exception:
                    pass

        for encoder in encoders.values():
            try:
                encoder.wait(timeout=5)
            except Exception:
                encoder.kill()

        for output_path in output_paths.values():
            try:
                if output_path.exists():
                    output_path.unlink()
            except Exception:
                pass

        raise

    for encoder in encoders.values():
        if encoder.stdin is not None:
            encoder.stdin.close()

    for camera_name, encoder in encoders.items():
        return_code = encoder.wait()

        if return_code != 0:
            raise RuntimeError(
                f"Encoding H.264 fallito per {camera_name} "
                f"(exit code {return_code})."
            )

    print()
    print("[OK] Estrazione completata.")

    for camera_name in CAMERAS:
        print(
            f"     {camera_name}: "
            f"{frame_counts[camera_name]} frame "
            f"@ {fps:.6g} FPS -> "
            f"{output_paths[camera_name].name}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Estrae i video RGB delle quattro camere da una trajectory .pkl "
            "e li salva accanto al file sorgente."
        )
    )

    parser.add_argument(
        "pkl_path",
        type=Path,
        help="Path del file trajectory .pkl.",
    )

    args = parser.parse_args()

    try:
        extract_videos(
            args.pkl_path.expanduser().resolve()
        )
    except Exception as exc:
        print(
            f"[ERROR] {type(exc).__name__}: {exc}"
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
