#!/usr/bin/env bash
set -Eeuo pipefail

PY=/opt/graspgenx_venv/bin/python
GX_ROOT=/opt/GraspGenX
REPO=/home/ros2_ws/src/UR5e-2f-85
PERSIST="$REPO/ai_controller/ai_controller/utils/grasping/graspgenx/persistent_worker.py"
SOCKET="${SEEDO_GRASPGENX_SOCKET:-/tmp/seedo_graspgenx_worker.sock}"
GEOMETRY_SOCKET="${SEEDO_GRASPGENX_GEOMETRY_SOCKET:-/tmp/seedo_graspgenx_geometry_worker.sock}"
export SEEDO_GRASPGENX_SOCKET="$SOCKET"
export SEEDO_GRASPGENX_GEOMETRY_SOCKET="$GEOMETRY_SOCKET"
SERVER_PID=
WORKER_PID=
GEOMETRY_PID=

# SeeDo memory-safe GraspGenX kNN runtime patch.
# No changes to the external GraspGenX repository.
run_gx_server() {
    exec "$PY" -u - "$GX_ROOT/client-server/graspgenx_server.py" "$@" <<'PY_GX_SERVER'
import os
import sys
import time
import runpy
import resource
from pathlib import Path

import numpy as np
import torch
from scipy.spatial import cKDTree

server_path = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(server_path.parent.parent))

from graspgenx.utils import point_cloud

original_knn = point_cloud.knn_points

SWITCH_N = 20
QUERY_CHUNK = 8192
WORKERS = int(os.environ.get("SEEDO_GRASPGENX_KNN_WORKERS", "4"))


def knn_points_kdtree(X, K, norm):
    started = time.perf_counter()

    points = np.ascontiguousarray(
        X.detach().numpy(),
        dtype=np.float64,
    )

    N = len(points)

    if not np.isfinite(points).all():
        raise ValueError("Non-finite GraspGenX point cloud")

    tree = cKDTree(points)

    all_dists = np.empty((N, K), dtype=np.float32)
    all_idxs = np.empty((N, K), dtype=np.int64)

    for start in range(0, N, QUERY_CHUNK):
        end = min(start + QUERY_CHUNK, N)

        dists, idxs = tree.query(
            points[start:end],
            k=K + 1,
            p=norm,
            eps=0.0,
            workers=WORKERS,
        )

        # Exclude the query point by index.
        # Preserve other points at zero distance.
        is_self = idxs == np.arange(start, end)[:, None]

        drop = np.where(
            is_self.any(axis=1),
            is_self.argmax(axis=1),
            K,
        )

        keep = np.arange(K + 1)[None, :] != drop[:, None]

        all_dists[start:end] = dists[keep].reshape(-1, K)
        all_idxs[start:end] = idxs[keep].reshape(-1, K)

    print(
        f"[SEEDO KNN] KDTree | N={N} | K={K} | "
        f"workers={WORKERS} | "
        f"time={time.perf_counter()-started:.4f}s",
        flush=True,
    )

    return torch.from_numpy(all_dists), torch.from_numpy(all_idxs)


def safe_knn_points(X, K, norm):
    if len(X) <= SWITCH_N:
        return original_knn(X, K, norm)

    supported = (
        X.ndim == 2
        and X.shape[1] == 3
        and X.device.type == "cpu"
        and X.dtype == torch.float32
        and not X.requires_grad
        and norm == 1
        and 0 < K < len(X)
    )

    if not supported:
        raise RuntimeError(
            "[SEEDO KNN] Unsupported large-cloud kNN. "
            "Quadratic allocation blocked."
        )

    print(
        f"[SEEDO KNN] START | "
        f"N={len(X)} | K={K} | PID={os.getpid()}",
        flush=True,
    )

    return knn_points_kdtree(X, K, norm)


point_cloud.knn_points = safe_knn_points

print(
    f"[SEEDO PATCH] Memory-safe kNN enabled | "
    f"switch={SWITCH_N} | workers={WORKERS} | "
    f"PID={os.getpid()}",
    flush=True,
)


def self_test():
    torch.manual_seed(42)
    torch.set_num_threads(4)

    # Compare both implementations on manageable inputs.
    for N in (2048, 4096, 8192):
        cloud = torch.rand(N, 3) * 0.1

        # Include duplicates and isolated outliers.
        cloud[:64] = cloud[64].clone()
        cloud[-32:] += 0.5

        t0 = time.perf_counter()
        old_dists, _ = original_knn(cloud, 20, 1)
        old_time = time.perf_counter() - t0

        t0 = time.perf_counter()
        new_dists, new_idxs = knn_points_kdtree(cloud, 20, 1)
        new_time = time.perf_counter() - t0

        old_mask = old_dists.mean(dim=1) < 0.014
        new_mask = new_dists.mean(dim=1) < 0.014

        mismatches = int((old_mask != new_mask).sum())
        max_diff = float((old_dists-new_dists).abs().max())

        assert torch.allclose(
            old_dists, new_dists,
            atol=1e-7, rtol=1e-6,
        )
        assert mismatches == 0
        assert not (
            new_idxs == torch.arange(N)[:, None]
        ).any()

        reconstructed = (
            cloud[new_idxs] - cloud[:, None, :]
        ).abs().sum(dim=-1)

        assert torch.allclose(
            reconstructed, new_dists,
            atol=1e-7, rtol=1e-6,
        )

        print(
            f"[TEST PASS] N={N} | "
            f"original={old_time:.4f}s | "
            f"kdtree={new_time:.4f}s | "
            f"max_diff={max_diff:.12f} | "
            f"mask_mismatches={mismatches}",
            flush=True,
        )

    # Stress test matching the latest failed request.
    N = 208513
    rng = np.random.default_rng(42)

    cloud = rng.uniform(
        -0.025, 0.025, (N, 3)
    ).astype(np.float32)

    face = rng.integers(0, 6, size=N)

    cloud[np.arange(N), face // 2] = np.where(
        face % 2 == 0, -0.025, 0.025
    )

    cloud[:, 2] += 0.06

    started = time.perf_counter()

    filtered, removed = point_cloud.point_cloud_outlier_removal(
        torch.from_numpy(cloud),
        threshold=0.014,
    )

    assert len(filtered) + len(removed) == N
    assert len(filtered) >= 10

    print(
        f"[STRESS PASS] N={N} | "
        f"kept={len(filtered)} | "
        f"removed={len(removed)} | "
        f"time={time.perf_counter()-started:.4f}s | "
        f"peak_RSS="
        f"{resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024:.2f} MiB",
        flush=True,
    )

    # Reuse the existing SeeDo warm-up for full offline inference.
    output = "/tmp/seedo_graspgenx_oom_replay.npz"

    np.savez(
        output,
        target_point_cloud_base=cloud,
    )

    print(f"[TEST] Replay saved: {output}", flush=True)
    print("ALL TESTS PASSED", flush=True)


if sys.argv[2:] == ["--self-test"]:
    self_test()
    sys.exit(0)

sys.argv = [str(server_path), *sys.argv[2:]]

runpy.run_path(
    str(server_path),
    run_name="__main__",
)
PY_GX_SERVER
}

# Offline kNN test without loading models or starting services.
if [[ "${SEEDO_GRASPGENX_KNN_TEST:-0}" == "1" ]]; then
    cd "$GX_ROOT"
    export PYTHONNOUSERSITE=1
    unset PYTHONPATH
    umask 077
    run_gx_server --self-test
fi


cleanup() {
    trap - EXIT
    if [[ -n "$GEOMETRY_PID" ]]; then
        kill "$GEOMETRY_PID" 2>/dev/null || true
    fi
    if [[ -n "$WORKER_PID" ]]; then
        kill "$WORKER_PID" 2>/dev/null || true
    fi
    if [[ -n "$SERVER_PID" ]]; then
        kill "$SERVER_PID" 2>/dev/null || true
    fi
    if [[ -n "$GEOMETRY_PID" ]]; then wait "$GEOMETRY_PID" 2>/dev/null || true; fi
    if [[ -n "$WORKER_PID" ]]; then wait "$WORKER_PID" 2>/dev/null || true; fi
    if [[ -n "$SERVER_PID" ]]; then wait "$SERVER_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for file in "$PY" "$GX_ROOT/client-server/graspgenx_server.py" "$PERSIST"; do
    [[ -f "$file" ]] || { echo "Missing: $file" >&2; exit 1; }
done
[[ "$SOCKET" != "$GEOMETRY_SOCKET" ]] || { echo "GPU and geometry socket paths must differ." >&2; exit 1; }
[[ ! -e "$SOCKET" ]] || { echo "Socket already exists: $SOCKET. Stop the old worker first." >&2; exit 1; }
[[ ! -e "$GEOMETRY_SOCKET" ]] || { echo "Socket already exists: $GEOMETRY_SOCKET. Stop the old geometry worker first." >&2; exit 1; }
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

run_gx_server \
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

# Geometry-selection worker uses the same Python interpreter and
# runtime_worker.run() as the previous per-request subprocess.
PYTHONPATH="$REPO/ai_controller" "$PY" -u -m \
    ai_controller.utils.grasping.graspgenx.geometry_persistent_worker &
GEOMETRY_PID=$!
echo "Starting persistent geometry worker (PID=$GEOMETRY_PID)..."
READY=0
for ((i=0; i<120; i++)); do
    kill -0 "$GEOMETRY_PID" 2>/dev/null || { echo "Geometry worker exited during startup." >&2; exit 1; }
    if [[ -S "$GEOMETRY_SOCKET" ]] && python3 - "$GEOMETRY_SOCKET" <<'PY_GEOMETRY_HEALTH'
import json, socket, sys
try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(0.8)
        s.connect(sys.argv[1])
        s.sendall(b'{"action":"health"}\n')
        with s.makefile('rb') as reader:
            response = json.loads(reader.readline(65536))
    assert response.get('ok') is True and response.get('status') == 'ready'
except (OSError, ValueError, AssertionError):
    sys.exit(1)
PY_GEOMETRY_HEALTH
    then
        READY=1
        break
    fi
    sleep 0.5
done
[[ "$READY" -eq 1 ]] || { echo "Geometry worker failed health/readiness check." >&2; exit 1; }

echo "[READY] GraspGenX server PID=$SERVER_PID; worker PID=$WORKER_PID; geometry PID=$GEOMETRY_PID"
echo "Run SeeDo with SEEDO_GRASPGENX_PERSISTENT=1 in its own process environment."
echo "Optionally set SEEDO_GRASPGENX_GEOMETRY_PERSISTENT=0 for a legacy-geometry A/B test."
echo "Stop all services with Ctrl+C."
wait -n "$SERVER_PID" "$WORKER_PID" "$GEOMETRY_PID"
