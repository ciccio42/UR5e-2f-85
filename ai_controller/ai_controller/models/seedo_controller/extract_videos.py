import argparse
import pickle
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np


CAMERAS = (
    "camera_front",
    "camera_lateral_left",
    "camera_lateral_right",
    "eye_in_hand",
)


class TrajectoryProxy:
    pass


class TrajectoryUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == "Trajectory":
            return TrajectoryProxy
        return super().find_class(module, name)


def decode_frame(image):
    image = np.asarray(image)

    if image.ndim == 1:
        frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

        if frame is None:
            raise ValueError("Impossibile decodificare il JPEG.")

        return frame

    if image.ndim == 3 and image.shape[2] == 3:
        return image

    raise ValueError(f"Formato immagine non valido: {image.shape}")


def create_encoder(output_path, width, height, fps):
    command = [
        "ffmpeg",
        "-y",
        "-loglevel", "error",
        "-f", "rawvideo",
        "-vcodec", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}",
        "-r", str(fps),
        "-i", "pipe:0",
        "-an",
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "20",
        "-pix_fmt", "yuv420p",
        "-movflags", "+faststart",
        str(output_path),
    ]

    return subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
    )


def extract_videos(pkl_path, fps):
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("FFmpeg non è installato.")

    with pkl_path.open("rb") as file:
        data = TrajectoryUnpickler(file).load()

    trajectory = data["traj"]

    if not trajectory._data:
        raise ValueError("La traiettoria è vuota.")

    encoders = {}
    frame_sizes = {}

    try:
        for step_index, entry in enumerate(trajectory._data):
            obs = entry[0]

            for camera in CAMERAS:
                key = f"{camera}_image"

                frame = np.ascontiguousarray(
                    decode_frame(obs[key])
                )

                height, width = frame.shape[:2]

                if camera not in encoders:
                    output_path = (
                        pkl_path.parent
                        / f"{pkl_path.stem}_{camera}.mp4"
                    )

                    encoders[camera] = create_encoder(
                        output_path,
                        width,
                        height,
                        fps,
                    )

                    frame_sizes[camera] = (width, height)

                    print(f"[INFO] Creazione: {output_path}")

                if (width, height) != frame_sizes[camera]:
                    raise ValueError(
                        f"Risoluzione variabile per {camera}."
                    )

                encoders[camera].stdin.write(
                    frame.tobytes()
                )

        print(
            f"\n[INFO] Elaborati {step_index + 1} "
            "frame per camera."
        )

    finally:
        for encoder in encoders.values():
            if encoder.stdin is not None:
                encoder.stdin.close()

        for camera, encoder in encoders.items():
            return_code = encoder.wait()

            if return_code != 0:
                raise RuntimeError(
                    f"Encoding H.264 fallito per {camera}."
                )

    print("[OK] Video H.264 salvati correttamente.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "pkl_path",
        type=Path,
    )

    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
    )

    args = parser.parse_args()

    if args.fps <= 0:
        parser.error("--fps deve essere maggiore di zero.")

    extract_videos(
        args.pkl_path.expanduser().resolve(),
        args.fps,
    )