import yaml
import numpy as np

def load_camera_calibration(calibration_path):
    """Load estimated_camera_positions.yaml: {camera_name: {position, orientation_matrix}}.

    Positions/orientations are expressed with respect to the raw ArUco marker
    origin/axes as returned by cv2.aruco's pose estimation (placed at the table
    center), per zed_camera/zed_camera_calibration/scripts/interactive_aruco_calibration.py.
    This is NOT necessarily the same frame as the ``table_0`` TF frame - see
    ARUCO_TO_TABLE0_ROTATION below for the fixed offset between the two.
    """
    with open(calibration_path, 'r') as f:
        raw = yaml.safe_load(f)

    calibration = {}
    for camera_name, entry in raw.items():
        calibration[camera_name] = {
            'position': np.array(entry['position'], dtype=np.float64),
            'orientation_matrix': np.array(entry['orientation_matrix'], dtype=np.float64),
        }
    return calibration

def robust_depth_at(depth_image, u, v, window=5):
    """Median depth (meters) in a small window around (u, v), ignoring NaN/inf/<=0."""
    h, w = depth_image.shape[:2]
    half = window // 2
    u0, u1 = max(0, u - half), min(w, u + half + 1)
    v0, v1 = max(0, v - half), min(h, v + half + 1)
    patch = np.asarray(depth_image[v0:v1, u0:u1], dtype=np.float64).flatten()
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))

def deproject_pixel(u, v, depth, camera_matrix):
    """Pinhole deprojection: pixel (u, v) + depth (m) -> 3D point in the camera
    optical frame (X right, Y down, Z forward), using intrinsics K."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    return np.array([x, y, depth], dtype=np.float64)

def camera_point_to_aruco(point_cam, camera_calib_entry):
    """Apply the camera->ArUco extrinsic transform loaded from the calibration
    yaml (camera position/orientation expressed in the raw ArUco marker frame)."""
    R_cm = camera_calib_entry['orientation_matrix']
    t_cm = camera_calib_entry['position']
    return R_cm @ point_cam + t_cm

# Fixed extrinsic offset between the raw ArUco marker origin/axes (as returned by
# cv2.aruco's pose estimation, i.e. the frame estimated_camera_positions.yaml is
# expressed in) and the table_0 TF frame: zero translation, quaternion (x, y, z, w)
# = (0, 0, 1, 0), i.e. a 180 degree rotation about Z (X and Y flip, Z unchanged).
ARUCO_TO_TABLE0_TRANSLATION = np.zeros(3)
ARUCO_TO_TABLE0_ROTATION = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, 1.0],
])

def aruco_point_to_table0(point_aruco):
    """Apply the fixed ArUco-origin -> table_0 transform (see
    ARUCO_TO_TABLE0_ROTATION above)."""
    return ARUCO_TO_TABLE0_ROTATION @ point_aruco + ARUCO_TO_TABLE0_TRANSLATION