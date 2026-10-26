import os
import threading

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