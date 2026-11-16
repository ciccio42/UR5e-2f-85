"""Long-lived local geometry-selection worker for GraspGenX.

This only hosts runtime_worker.run(), the same algorithm as the previous
per-inference subprocess. It never sends robot commands.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
from pathlib import Path
import signal
import socket
import time
import traceback

from .runtime_worker import run

DEFAULT_SOCKET = "/tmp/seedo_graspgenx_geometry_worker.sock"
MAX_REQUEST_BYTES = 65_536


def handle_request(connection: socket.socket) -> None:
    with connection:
        try:
            with connection.makefile("rb") as stream:
                raw = stream.readline(MAX_REQUEST_BYTES + 1)
            if not raw or len(raw) > MAX_REQUEST_BYTES or not raw.endswith(b"\n"):
                raise ValueError("Invalid geometry-worker request framing")

            request = json.loads(raw.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("Expected JSON request object")

            action = request.get("action")
            if action == "health":
                response = {"ok": True, "pid": os.getpid(), "status": "ready"}
            elif action == "select":
                output = io.StringIO()
                started = time.perf_counter()
                with contextlib.redirect_stdout(output):
                    run(
                        input_path=Path(request["input"]),
                        output_path=Path(request["output"]),
                        table_margin_mm=float(request["table_margin_mm"]),
                        semantic_radius_px=float(request["semantic_radius_px"]),
                    )
                response = {
                    "ok": True,
                    "pid": os.getpid(),
                    "stdout": output.getvalue(),
                    "compute_s": time.perf_counter() - started,
                }
            else:
                raise ValueError(f"Unknown geometry-worker action: {action!r}")
        except (Exception, SystemExit):
            response = {
                "ok": False,
                "pid": os.getpid(),
                "error": traceback.format_exc(),
            }

        try:
            connection.sendall((json.dumps(response) + "\n").encode("utf-8"))
        except OSError:
            pass  # Client may have timed out; daemon stays available.


def shutdown(_signum, _frame):
    raise SystemExit(0)


def main() -> None:
    signal.signal(signal.SIGTERM, shutdown)
    socket_path = Path(os.environ.get("SEEDO_GRASPGENX_GEOMETRY_SOCKET", DEFAULT_SOCKET))
    if socket_path.exists():
        raise RuntimeError(f"Geometry socket already exists: {socket_path}")

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        server.bind(str(socket_path))
        os.chmod(socket_path, 0o600)
        server.listen(8)
        print(
            f"[GRASPGENX GEOMETRY] Ready | PID={os.getpid()} | socket={socket_path}",
            flush=True,
        )
        while True:
            connection, _ = server.accept()
            handle_request(connection)
    finally:
        server.close()
        socket_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
