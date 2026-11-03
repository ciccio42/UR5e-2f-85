import os
import shutil
import subprocess
import tempfile
import threading

from pathlib import Path

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_FRONT_CAMERA_TOPIC = (
    "/zed_front/zed_node/rgb/color/rect/image"
)


class FrontCameraStream:

    def __init__(self):

        self.topic = os.environ.get(
            "SEEDO_GUI_FRONT_CAMERA_TOPIC",
            DEFAULT_FRONT_CAMERA_TOPIC,
        )

        self._lock = threading.Lock()
        self._condition = threading.Condition(
            self._lock
        )

        self._latest_jpeg = None
        self._sequence = 0

        self._context = None
        self._node = None
        self._executor = None
        self._thread = None

        self._stop_event = threading.Event()
        self._started = False

        self._bridge = CvBridge()

        # -----------------------------------------------------
        # Temporary robot-execution video recording
        # -----------------------------------------------------

        self._recording_lock = threading.RLock()

        self._recording_requested = False
        self._recording_process = None

        self._recording_temp_path = None

        self._recording_width = None
        self._recording_height = None

        self._recording_frame_count = 0

        self._recording_fps = float(
            os.environ.get(
                "SEEDO_GUI_RECORDING_FPS",
                "11",
            )
        )

        self._recording_error = None


    def start(self):

        with self._lock:

            if self._started:
                return

            self._started = True


        self._context = Context()

        rclpy.init(
            context=self._context
        )


        self._node = Node(
            "seedo_gui_front_camera",
            context=self._context,
        )


        self._node.create_subscription(
            Image,
            self.topic,
            self._image_callback,
            qos_profile_sensor_data,
        )


        self._executor = SingleThreadedExecutor(
            context=self._context
        )

        self._executor.add_node(
            self._node
        )


        self._thread = threading.Thread(
            target=self._spin,
            daemon=True,
        )

        self._thread.start()


    def _spin(self):

        while (
            not self._stop_event.is_set()
            and self._context is not None
            and self._context.ok()
        ):

            self._executor.spin_once(
                timeout_sec=0.1
            )


    def _image_callback(
        self,
        message: Image,
    ):

        try:

            frame = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )

            self._record_frame(
                frame
            )

            success, encoded = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    80,
                ],
            )


            if not success:
                return


            jpeg = encoded.tobytes()


            with self._condition:

                self._latest_jpeg = jpeg

                self._sequence += 1

                self._condition.notify_all()

        except Exception:
            return

    def _start_ffmpeg_locked(
        self,
        frame,
    ) -> None:

        if self._recording_process is not None:
            return

        height, width = frame.shape[:2]

        self._recording_width = width
        self._recording_height = height


        temp_file = tempfile.NamedTemporaryFile(
            prefix="seedo_gui_robot_execution_",
            suffix=".mp4",
            delete=False,
        )

        temp_path = Path(
            temp_file.name
        )

        temp_file.close()


        command = [
            "/usr/bin/ffmpeg",
            "-y",

            "-loglevel",
            "error",

            "-f",
            "rawvideo",

            "-pix_fmt",
            "bgr24",

            "-s:v",
            f"{width}x{height}",

            "-r",
            str(self._recording_fps),

            "-i",
            "-",

            "-an",

            "-c:v",
            "libx264",

            "-preset",
            "veryfast",

            "-crf",
            "18",

            "-pix_fmt",
            "yuv420p",

            "-movflags",
            "+faststart",

            str(temp_path),
        ]


        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        except Exception:
            try:
                temp_path.unlink(
                    missing_ok=True
                )
            except Exception:
                pass

            raise


        self._recording_process = process
        self._recording_temp_path = temp_path

    def _record_frame(
        self,
        frame,
    ) -> None:

        with self._recording_lock:

            if not self._recording_requested:
                return


            try:
                if self._recording_process is None:
                    self._start_ffmpeg_locked(
                        frame
                    )


                process = (
                    self._recording_process
                )

                if (
                    process is None
                    or process.stdin is None
                ):
                    return


                if process.poll() is not None:
                    self._recording_error = (
                        "FFmpeg recording process "
                        "terminated unexpectedly."
                    )

                    self._recording_requested = False

                    return


                process.stdin.write(
                    frame.tobytes()
                )

                self._recording_frame_count += 1


            except Exception as exc:
                self._recording_error = str(
                    exc
                )

                self._recording_requested = False

    def begin_recording(
        self,
    ) -> None:

        self.start()

        with self._recording_lock:

            # A new robot execution must never reuse
            # a previous temporary recording.
            self._discard_recording_locked()

            self._recording_requested = True

            self._recording_frame_count = 0

            self._recording_width = None
            self._recording_height = None

            self._recording_error = None

    def _finalize_recording_locked(
        self,
    ) -> None:

        self._recording_requested = False

        process = self._recording_process

        if process is None:
            return


        try:
            if process.stdin is not None:
                process.stdin.close()

        except Exception:
            pass


        try:
            process.wait(
                timeout=10
            )

        except subprocess.TimeoutExpired:

            try:
                process.kill()
            except Exception:
                pass

            try:
                process.wait(
                    timeout=2
                )
            except Exception:
                pass


        self._recording_process = None

    def end_recording(
        self,
    ) -> None:

        with self._recording_lock:
            self._finalize_recording_locked()

    def _discard_recording_locked(
        self,
    ) -> None:

        self._finalize_recording_locked()

        temp_path = (
            self._recording_temp_path
        )

        if temp_path is not None:

            try:
                temp_path.unlink(
                    missing_ok=True
                )

            except Exception:
                pass


        self._recording_temp_path = None

        self._recording_frame_count = 0

        self._recording_width = None
        self._recording_height = None

    def discard_recording(
        self,
    ) -> None:

        with self._recording_lock:
            self._discard_recording_locked()

    def commit_recording(
        self,
        artifact_root: str | Path,
    ) -> Path:

        with self._recording_lock:

            self._finalize_recording_locked()

            temp_path = (
                self._recording_temp_path
            )


            if (
                temp_path is None
                or not temp_path.is_file()
            ):
                raise RuntimeError(
                    "No robot execution recording "
                    "is available to save."
                )


            if temp_path.stat().st_size == 0:
                raise RuntimeError(
                    "Robot execution recording is empty."
                )


            artifact_root = (
                Path(artifact_root)
                .expanduser()
                .resolve()
            )


            rollout_dir = (
                artifact_root
                / "rollouts"
            )

            rollout_dir.mkdir(
                parents=True,
                exist_ok=True,
            )


            output_path = (
                rollout_dir
                / "robot_execution_front_camera.mp4"
            )


            if output_path.exists():
                output_path.unlink()


            shutil.move(
                str(temp_path),
                str(output_path),
            )


            self._recording_temp_path = None

            return output_path

    def get_recording_status(
        self,
    ) -> dict:

        with self._recording_lock:

            temp_path = (
                self._recording_temp_path
            )

            return {
                "recording":
                    self._recording_requested,

                "finalized":
                    (
                        not self._recording_requested
                        and temp_path is not None
                        and temp_path.is_file()
                    ),

                "frame_count":
                    self._recording_frame_count,

                "width":
                    self._recording_width,

                "height":
                    self._recording_height,

                "fps":
                    self._recording_fps,

                "temp_path":
                    (
                        str(temp_path)
                        if temp_path is not None
                        else None
                    ),

                "error":
                    self._recording_error,
            }

    def mjpeg_frames(self):

        self.start()

        last_sequence = -1


        while not self._stop_event.is_set():

            with self._condition:

                self._condition.wait_for(
                    lambda: (
                        self._sequence
                        != last_sequence
                        or self._stop_event.is_set()
                    ),
                    timeout=1.0,
                )


                if self._stop_event.is_set():
                    break


                if (
                    self._latest_jpeg is None
                    or self._sequence
                    == last_sequence
                ):
                    continue


                frame = self._latest_jpeg

                last_sequence = (
                    self._sequence
                )


            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-cache\r\n"
                b"\r\n"
                + frame
                + b"\r\n"
            )


    def stop(self):

        self.discard_recording()

        self._stop_event.set()

        with self._condition:
            self._condition.notify_all()


        if self._executor is not None:

            try:
                self._executor.shutdown()

            except Exception:
                pass


        if self._node is not None:

            try:
                self._node.destroy_node()

            except Exception:
                pass


        if (
            self._context is not None
            and self._context.ok()
        ):

            try:
                self._context.shutdown()

            except Exception:
                pass


FRONT_CAMERA_STREAM = FrontCameraStream()