import json
import os
import signal
import subprocess
import threading

from collections import deque
from copy import deepcopy
from pathlib import Path


from ai_controller.utils.gui.interaction_bridge import (
    GUI_INTERACTION_PREFIX,
)

from ai_controller.utils.gui.pipeline_bridge import (
    PIPELINE_EVENT_PREFIX,
)

from ai_controller.utils.gui.backend.interaction_manager import (
    INTERACTION_MANAGER,
    InteractionManager,
)

from ai_controller.utils.gui.backend.event_manager import (
    EVENT_MANAGER,
)


class ProcessManager:

    def __init__(
        self,
        interaction_manager: InteractionManager = INTERACTION_MANAGER,
        max_logs: int = 5000,
    ):
        self._lock = threading.Lock()

        self._interaction_manager = (
            interaction_manager
        )

        self._process = None

        self._reader_thread = None

        self._logs = deque(
            maxlen=max_logs
        )

        self._log_sequence = 0

        self._status = "idle"

        self._return_code = None

        self._command = None

        self._last_error = None

        self._failure_candidate = None


    def start(
        self,
        command: list[str],
        cwd: str | Path | None = None,
        extra_env: dict | None = None,
    ) -> dict:

        with self._lock:

            if (
                self._process is not None
                and self._process.poll() is None
            ):
                raise RuntimeError(
                    "A process is already running."
                )


            self._logs.clear()

            self._log_sequence = 0

            self._status = "starting"

            self._return_code = None

            self._command = list(
                command
            )

            self._last_error = None

            self._failure_candidate = None


        self._interaction_manager.clear()

        EVENT_MANAGER.clear()


        env = os.environ.copy()

        # Enabled ONLY for processes launched
        # through the dashboard.
        env[
            "SEEDO_GUI_INTERACTION"
        ] = "1"

        env[
            "SEEDO_GUI_TELEMETRY"
        ] = "1"


        if extra_env:
            env.update(
                extra_env
            )


        try:
            process = subprocess.Popen(
                command,

                stdin=subprocess.PIPE,

                stdout=subprocess.PIPE,

                stderr=subprocess.STDOUT,

                text=True,

                bufsize=1,

                cwd=(
                    str(cwd)
                    if cwd is not None
                    else None
                ),

                env=env,

                start_new_session=True,
            )

        except Exception as exc:

            with self._lock:
                self._status = "failed"

                self._last_error = str(
                    exc
                )

            raise


        with self._lock:
            self._process = process

            self._status = "running"


        self._reader_thread = threading.Thread(
            target=self._read_output,
            args=(process,),
            daemon=True,
        )

        self._reader_thread.start()


        return self.get_status()


    def _read_output(
        self,
        process: subprocess.Popen,
    ) -> None:

        try:

            if process.stdout is None:
                raise RuntimeError(
                    "Process stdout is unavailable."
                )


            for raw_line in process.stdout:

                line = raw_line.rstrip(
                    "\r\n"
                )


                # =================================================
                # GUI USER INTERACTION
                # =================================================

                if line.startswith(
                    GUI_INTERACTION_PREFIX
                ):

                    payload_text = line[
                        len(
                            GUI_INTERACTION_PREFIX
                        ):
                    ]


                    try:
                        event = json.loads(
                            payload_text
                        )

                        if (
                            event.get("event")
                            != "user_interaction_required"
                        ):
                            raise ValueError(
                                "Unsupported GUI interaction event."
                            )


                        self._interaction_manager.register(
                            interaction=event,
                            response_handler=self._send_stdin,
                        )


                    except Exception as exc:

                        self._append_log(
                            "[GUI INTERACTION ERROR] "
                            + str(exc)
                        )


                    continue


                # =================================================
                # PIPELINE TELEMETRY
                # =================================================

                if line.startswith(
                    PIPELINE_EVENT_PREFIX
                ):

                    payload_text = line[
                        len(
                            PIPELINE_EVENT_PREFIX
                        ):
                    ]


                    try:
                        event = json.loads(
                            payload_text
                        )

                        EVENT_MANAGER.register(
                            event
                        )


                    except Exception as exc:

                        self._append_log(
                            "[PIPELINE EVENT ERROR] "
                            + str(exc)
                        )


                    continue


                # =================================================
                # NORMAL PROCESS OUTPUT
                # =================================================

                self._append_log(
                    line
                )

                self._remember_failure_candidate(
                    line
                )


            return_code = process.wait()


            self._interaction_manager.clear()


            with self._lock:

                self._return_code = (
                    return_code
                )


                if self._status == "stopping":
                    self._status = "stopped"

                elif return_code == 0:
                    self._status = "completed"

                else:
                    self._status = "failed"

                    if self._last_error is None:
                        self._last_error = (
                            self._failure_candidate
                            or
                            (
                                "Experiment process exited "
                                f"with code {return_code}."
                            )
                        )


        except Exception as exc:

            self._interaction_manager.clear()


            with self._lock:

                self._last_error = str(
                    exc
                )


                return_code = (
                    process.poll()
                )


                if return_code is not None:
                    self._return_code = (
                        return_code
                    )


                if self._status == "stopping":
                    self._status = "stopped"

                else:
                    self._status = "failed"


    def _append_log(
        self,
        line: str,
    ) -> None:

        with self._lock:

            self._log_sequence += 1


            self._logs.append(
                {
                    "sequence":
                        self._log_sequence,

                    "line":
                        line,
                }
            )

    def _remember_failure_candidate(
        self,
        line: str,
    ) -> None:

        stripped = line.strip()

        error_prefixes = (
            "RuntimeError:",
            "ValueError:",
            "TypeError:",
            "ModuleNotFoundError:",
            "FileNotFoundError:",
            "ImportError:",
            "Exception:",
            "ERROR:",
            "[ERROR]",
        )

        if not stripped.startswith(
            error_prefixes
        ):
            return

        with self._lock:
            self._failure_candidate = stripped


    def _send_stdin(
        self,
        value: str,
    ) -> None:

        with self._lock:
            process = self._process


        if (
            process is None
            or process.poll() is not None
        ):
            raise RuntimeError(
                "Cannot send input: "
                "the process is not running."
            )


        if process.stdin is None:
            raise RuntimeError(
                "Cannot send input: "
                "process stdin is unavailable."
            )


        process.stdin.write(
            value + "\n"
        )

        process.stdin.flush()


    def stop(
        self,
    ) -> None:

        with self._lock:

            process = self._process


            if (
                process is None
                or process.poll() is not None
            ):
                return


            self._status = "stopping"


        try:
            process_group = os.getpgid(
                process.pid
            )

            os.killpg(
                process_group,
                signal.SIGINT,
            )


        except ProcessLookupError:
            pass


        except Exception as exc:

            with self._lock:
                self._last_error = str(
                    exc
                )

            raise


    def get_status(
        self,
    ) -> dict:

        with self._lock:

            pid = (
                self._process.pid
                if self._process is not None
                else None
            )


            return {
                "status":
                    self._status,

                "pid":
                    pid,

                "return_code":
                    self._return_code,

                "command":
                    deepcopy(
                        self._command
                    ),

                "last_error":
                    self._last_error,
            }


    def get_logs(
        self,
        after_sequence: int = 0,
    ) -> list[dict]:

        with self._lock:

            return [
                deepcopy(entry)

                for entry in self._logs

                if entry["sequence"]
                > after_sequence
            ]


PROCESS_MANAGER = ProcessManager()