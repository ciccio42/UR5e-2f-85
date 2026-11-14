#!/usr/bin/env bash
set -Eeuo pipefail

PY=/opt/graspgenx_venv/bin/python
GX_ROOT=/opt/GraspGenX
REPO=/home/ros2_ws/src/UR5e-2f-85
PERSIST="$REPO/ai_controller/ai_controller/utils/grasping/graspgenx/persistent_worker.py"
SOCKET=/tmp/seedo_graspgenx_worker.sock
SERVER_PID=
WORKER_PID=

cleanup() {
    trap - EXIT
    if [[ -n "$WORKER_PID" ]]; then
        kill "$WORKER_PID" 2>/dev/null || true
    fi
    if [[ -n "$SERVER_PID" ]]; then
        kill "$SERVER_PID" 2>/dev/null || true
    fi
    if [[ -n "$WORKER_PID" ]]; then wait "$WORKER_PID" 2>/dev/null || true; fi
    if [[ -n "$SERVER_PID" ]]; then wait "$SERVER_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for file in "$PY" "$GX_ROOT/client-server/graspgenx_server.py" "$PERSIST"; do
    [[ -f "$file" ]] || { echo "Missing: $file" >&2; exit 1; }
done
[[ ! -e "$SOCKET" ]] || { echo "Socket already exists: $SOCKET. Stop the old worker first." >&2; exit 1; }
if python3 - <<'PY'
import socket, sys
with socket.socket() as s:
    s.settimeout(0.2)
    sys.exit(0 if s.connect_ex(("127.0.0.1", 8782)) == 0 else 1)
PY
then
    echo "Port 8782 is already in use. Stop the old GraspGenX server first." >&2
    exit 1
fi

cd "$GX_ROOT"
export PYTHONNOUSERSITE=1
export GRASPGENX_CHECKPOINT_DIR=/opt/graspgenx_checkpoints
export GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx_gripper_descriptions
unset PYTHONPATH
umask 077

"$PY" -u client-server/graspgenx_server.py \
    --config /opt/graspgenx_checkpoints/release \
    --assets_dir /opt/graspgenx_gripper_descriptions/gripper_descriptions/assets \
    --default_gripper robotiq_2f_85 \
    --host 0.0.0.0 --port 8782 &
SERVER_PID=$!

echo "Starting GraspGenX GPU server (PID=$SERVER_PID)..."
READY=0
for ((i=0; i<240; i++)); do
    kill -0 "$SERVER_PID" 2>/dev/null || { echo "Server exited during startup." >&2; exit 1; }
    if python3 - <<'PY'
import socket, sys
with socket.socket() as s:
    s.settimeout(0.2)
    sys.exit(0 if s.connect_ex(("127.0.0.1", 8782)) == 0 else 1)
PY
    then
        READY=1
        break
    fi
    sleep 0.5
done
[[ "$READY" -eq 1 ]] || { echo "Server did not open port 8782." >&2; exit 1; }

"$PY" -u "$PERSIST" &
WORKER_PID=$!
echo "Starting persistent worker (PID=$WORKER_PID)..."
READY=0
for ((i=0; i<120; i++)); do
    kill -0 "$WORKER_PID" 2>/dev/null || { echo "Worker exited during startup." >&2; exit 1; }
    if [[ -S "$SOCKET" ]] && python3 - "$SOCKET" <<'PY'
import json, socket, sys
try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(0.8)
        s.connect(sys.argv[1])
        s.sendall(b'["--action","health","--host","127.0.0.1","--port","8782"]\n')
        with s.makefile("rb") as reader:
            response = json.loads(reader.readline(65536))
    assert response.get("ok") is True
    assert '"status": "ok"' in response.get("stdout", "")
except (OSError, ValueError, AssertionError):
    sys.exit(1)
PY
    then
        READY=1
        break
    fi
    sleep 0.5
done
[[ "$READY" -eq 1 ]] || { echo "Worker failed health/readiness check." >&2; exit 1; }

# Warm the same model path used by SeeDo: infer_object / graspmoe / dense OBB.
# This is a single synthetic inference, not a grasp sent to the robot.
# Set SEEDO_GRASPGENX_WARMUP=0 to skip it for troubleshooting.
if [[ "${SEEDO_GRASPGENX_WARMUP:-1}" == "1" ]]; then
    echo "[WARMUP] Running one GraspGenX infer_object through the persistent worker..."
    python3 - "$SOCKET" <<'PY_WARMUP'
import json
import os
import socket
import sys
import tempfile
import time
from pathlib import Path

import numpy as np


def get_point_cloud():
    # Optional: use a previously recorded and valid SeeDo target cloud.
    # Otherwise use an independent, deterministic cuboid surface cloud.
    diagnostics = os.environ.get("SEEDO_GRASPGENX_WARMUP_DIAGNOSTICS", "")
    if diagnostics:
        with np.load(diagnostics, allow_pickle=False) as saved:
            cloud = np.asarray(saved["target_point_cloud_base"], dtype=np.float32)
        source = diagnostics
    else:
        rng = np.random.default_rng(42)
        cloud = rng.uniform(-0.025, 0.025, (2048, 3)).astype(np.float32)
        face = rng.integers(0, 6, size=len(cloud))
        axis = face // 2
        cloud[np.arange(len(cloud)), axis] = np.where(
            face % 2 == 0, -0.025, 0.025
        )
        cloud[:, 2] += 0.06
        source = "synthetic 5-cm cuboid (2048 points)"
    if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) == 0:
        raise ValueError(f"Invalid warm-up point cloud shape: {cloud.shape}")
    if not np.isfinite(cloud).all():
        raise ValueError("Warm-up point cloud contains NaN or Inf")
    return cloud, source


cloud, source = get_point_cloud()
print(f"[WARMUP] Point cloud: {source}; N={len(cloud)}", flush=True)
started = time.perf_counter()

with tempfile.TemporaryDirectory(prefix="graspgenx_warmup_") as td:
    input_path = str(Path(td) / "request.npz")
    output_path = str(Path(td) / "response.npz")
    np.savez(
        input_path,
        point_cloud=cloud,
        extents_open=np.array([0.085, 0.032, 0.036], dtype=np.float32),
        offset_open=np.array([0.0, 0.0, 0.130], dtype=np.float32),
        extents_mid=np.array([0.046, 0.032, 0.036], dtype=np.float32),
        offset_mid=np.array([0.0, 0.0, 0.143], dtype=np.float32),
        gripper_type=np.array(0, dtype=np.int32),
        fingertip_depth=np.array(0.136, dtype=np.float32),
    )

    arguments = [
        "--action", "infer_object",
        "--host", "127.0.0.1", "--port", "8782",
        "--input", input_path, "--output", output_path,
        "--planner", "graspmoe", "--num-grasps", "200",
        "--grasp-threshold", "-1.0", "--topk-num-grasps", "-1",
        "--obb-density", "dense", "--obb-position-spacing-m", "0.01",
    ]
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(180.0)
        conn.connect(sys.argv[1])
        conn.sendall((json.dumps(arguments) + "\n").encode("utf-8"))
        with conn.makefile("rb") as stream:
            raw = stream.readline(1_048_577)
    if not raw or not raw.endswith(b"\n") or len(raw) > 1_048_576:
        raise RuntimeError("Incomplete/oversized persistent-worker warm-up reply")
    response = json.loads(raw)
    if not isinstance(response, dict):
        raise RuntimeError("Invalid warm-up response type")
    if response.get("ok") is not True:
        raise RuntimeError(f"GraspGenX warm-up failed: {response.get('error', response)}")
    if not Path(output_path).is_file():
        raise RuntimeError("GraspGenX warm-up did not write response.npz")
    with np.load(output_path, allow_pickle=False) as result:
        grasps = result["grasps"]
        scores = result["confidences"]
        tags = result["branch_tags"]
        if grasps.ndim != 3 or grasps.shape[1:] != (4, 4):
            raise RuntimeError(f"Invalid warm-up grasp shape: {grasps.shape}")
        if len(scores) != len(grasps) or len(tags) != len(grasps):
            raise RuntimeError("Inconsistent warm-up output array lengths")
        count = len(grasps)

print(f"[WARMUP] Completed in {time.perf_counter()-started:.3f} s | grasp candidates={count} (discarded)", flush=True)
PY_WARMUP
else
    echo "[WARMUP] Disabled (SEEDO_GRASPGENX_WARMUP=0)."
fi

echo "[READY] GraspGenX server PID=$SERVER_PID; worker PID=$WORKER_PID"
echo "Run SeeDo with SEEDO_GRASPGENX_PERSISTENT=1 in its own process environment."
echo "Stop both services with Ctrl+C."
wait -n "$SERVER_PID" "$WORKER_PID"
