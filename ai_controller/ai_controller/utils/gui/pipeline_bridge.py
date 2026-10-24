import json
import os


PIPELINE_EVENT_PREFIX = "__SEEDO_PIPELINE_EVENT__"


def gui_telemetry_enabled() -> bool:
    value = os.environ.get(
        "SEEDO_GUI_TELEMETRY",
        "",
    ).strip().lower()

    return value in {
        "1",
        "true",
        "yes",
        "on",
    }


def emit_pipeline_event(
    event: str,
    **data,
) -> None:
    """
    Emit a structured event for the SeeDo GUI.

    When the process is launched normally from terminal,
    telemetry is disabled and this function does nothing.
    """

    if not gui_telemetry_enabled():
        return

    payload = {
        "event": event,
        **data,
    }

    print(
        PIPELINE_EVENT_PREFIX
        + json.dumps(
            payload,
            ensure_ascii=False,
        ),
        flush=True,
    )