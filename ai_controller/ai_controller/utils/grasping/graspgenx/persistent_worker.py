
from __future__ import annotations

import contextlib
import io
import json
import os
import signal
import socket
import traceback
from pathlib import Path

from worker import main as worker_main


SOCKET_PATH = Path("/tmp/seedo_graspgenx_worker.sock")
MAX_REQUEST_BYTES = 65536


def handle_request(conn: socket.socket) -> None:
    with conn:
        try:
            with conn.makefile("rb") as stream:
                raw = stream.readline(MAX_REQUEST_BYTES + 1)

            if (
                not raw
                or len(raw) > MAX_REQUEST_BYTES
                or not raw.endswith(b"\n")
            ):
                raise ValueError("Invalid request framing.")

            arguments = json.loads(raw)

            if (
                not isinstance(arguments, list)
                or not all(isinstance(x, str) for x in arguments)
            ):
                raise ValueError("Expected a list of arguments.")

            output = io.StringIO()

            # The existing worker performs the actual inference.
            # Each invocation creates a fresh ZMQ client.
            with contextlib.redirect_stdout(output):
                worker_main(arguments)

            response = {
                "ok": True,
                "pid": os.getpid(),
                "stdout": output.getvalue(),
            }

        except (Exception, SystemExit):
            response = {
                "ok": False,
                "pid": os.getpid(),
                "error": traceback.format_exc(),
            }

        try:
            conn.sendall(
                json.dumps(response).encode("utf-8") + b"\n"
            )
        except OSError:
            # The caller may have closed its connection.
            # The persistent worker remains available.
            pass


def shutdown(signum, frame):
    raise SystemExit(0)


def main() -> None:
    signal.signal(signal.SIGTERM, shutdown)

    if SOCKET_PATH.exists():
        raise RuntimeError(
            f"Socket already exists: {SOCKET_PATH}. "
            "Check whether another worker is running."
        )

    server = socket.socket(
        socket.AF_UNIX,
        socket.SOCK_STREAM,
    )

    try:
        server.bind(str(SOCKET_PATH))
        os.chmod(SOCKET_PATH, 0o600)
        server.listen(8)

        print(
            f"[GRASPGENX PERSISTENT] Ready | "
            f"PID={os.getpid()} | "
            f"socket={SOCKET_PATH}",
            flush=True,
        )

        while True:
            conn, _ = server.accept()
            handle_request(conn)

    finally:
        server.close()
        SOCKET_PATH.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
