import json
import os
import sys
import uuid


GUI_INTERACTION_PREFIX = "__SEEDO_GUI_INTERACTION__"


def gui_interaction_enabled() -> bool:
    value = os.environ.get(
        "SEEDO_GUI_INTERACTION",
        "",
    ).strip().lower()

    return value in {
        "1",
        "true",
        "yes",
        "on",
    }


def request_user_input(
    prompt: str,
    interaction_type: str = "text",
) -> str:
    """
    Request user input.

    Terminal mode:
        behaves exactly like Python's built-in input().

    GUI mode:
        emits a structured interaction event on stdout and waits
        for the GUI backend to provide the answer through stdin.

    The returned value is always the raw string supplied by the user.
    Interpretation and validation remain the responsibility of the
    existing caller in ai_controller_node.py.
    """

    if not gui_interaction_enabled():
        return input(prompt)

    interaction_id = uuid.uuid4().hex

    event = {
        "event": "user_interaction_required",
        "interaction_id": interaction_id,
        "prompt": prompt,
        "interaction_type": interaction_type,
    }

    print(
        GUI_INTERACTION_PREFIX
        + json.dumps(
            event,
            ensure_ascii=False,
        ),
        flush=True,
    )

    response = sys.stdin.readline()

    if response == "":
        raise EOFError(
            "GUI interaction stdin was closed while waiting "
            f"for interaction {interaction_id}."
        )

    return response.rstrip("\r\n")