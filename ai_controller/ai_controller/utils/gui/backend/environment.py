import os


def detect_execution_mode() -> tuple[str, str]:
    """
    Detect whether the GUI is running in the real-robot or Isaac container.

    Priority:
    1. Explicit SEEDO_EXECUTION_MODE
    2. Presence of ROBOT_IP
    3. Otherwise assume Isaac
    """
    explicit_mode = os.environ.get(
        "SEEDO_EXECUTION_MODE",
        "",
    ).strip().lower()

    if explicit_mode in {"real", "isaac"}:
        return explicit_mode, "SEEDO_EXECUTION_MODE"

    if os.environ.get("ROBOT_IP", "").strip():
        return "real", "ROBOT_IP"

    return "isaac", "no ROBOT_IP"


def ros_available() -> bool:
    try:
        import rclpy  # noqa: F401

        return True
    except Exception:
        return False