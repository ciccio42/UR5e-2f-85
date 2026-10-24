from __future__ import annotations

from copy import deepcopy
from threading import RLock
from typing import Callable


ResponseHandler = Callable[[str], None]


class InteractionManager:
    """
    Stores the user interaction currently requested by the running
    controller process.

    The manager does not interpret prompts or responses.

    It only:
    - receives an interaction;
    - exposes it to the GUI;
    - receives the raw user response;
    - forwards it to the process.
    """

    def __init__(self) -> None:
        self._lock = RLock()

        self._pending: dict | None = None
        self._response_handler: ResponseHandler | None = None

    def register(
        self,
        interaction: dict,
        response_handler: ResponseHandler,
    ) -> None:
        interaction_id = interaction.get("interaction_id")
        prompt = interaction.get("prompt")

        if not interaction_id:
            raise ValueError(
                "Interaction is missing 'interaction_id'."
            )

        if prompt is None:
            raise ValueError(
                "Interaction is missing 'prompt'."
            )

        with self._lock:
            if self._pending is not None:
                raise RuntimeError(
                    "Another user interaction is already pending."
                )

            self._pending = deepcopy(interaction)
            self._response_handler = response_handler

    def get_pending(self) -> dict | None:
        with self._lock:
            if self._pending is None:
                return None

            return deepcopy(self._pending)

    def respond(
        self,
        interaction_id: str,
        value: str,
    ) -> None:
        with self._lock:
            if self._pending is None:
                raise LookupError(
                    "There is no pending user interaction."
                )

            current_id = self._pending.get(
                "interaction_id"
            )

            if current_id != interaction_id:
                raise LookupError(
                    f"Interaction '{interaction_id}' is not pending."
                )

            response_handler = self._response_handler

            if response_handler is None:
                raise RuntimeError(
                    "Pending interaction has no response handler."
                )

            # Clear before forwarding the response.
            #
            # The process may immediately generate another interaction
            # after receiving this answer.
            self._pending = None
            self._response_handler = None

        try:
            response_handler(value)

        except Exception:
            # Restore the interaction if forwarding failed.
            with self._lock:
                self._pending = {
                    "event": "user_interaction_required",
                    "interaction_id": interaction_id,
                    "prompt": "Failed to forward previous interaction.",
                }
                self._response_handler = response_handler

            raise

    def clear(self) -> None:
        with self._lock:
            self._pending = None
            self._response_handler = None


INTERACTION_MANAGER = InteractionManager()