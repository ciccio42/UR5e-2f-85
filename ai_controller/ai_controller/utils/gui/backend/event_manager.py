from collections import deque
from copy import deepcopy
from threading import Lock


class EventManager:

    def __init__(
        self,
        max_events: int = 5000,
    ):
        self._lock = Lock()

        self._events = deque(
            maxlen=max_events
        )

        self._sequence = 0


    def register(
        self,
        event: dict,
    ) -> dict:
        if not isinstance(event, dict):
            raise ValueError(
                "Pipeline event must be a dictionary."
            )

        event_type = event.get(
            "event"
        )

        if not isinstance(
            event_type,
            str,
        ) or not event_type:
            raise ValueError(
                "Pipeline event requires a valid 'event' field."
            )


        with self._lock:
            self._sequence += 1

            stored_event = {
                "sequence": self._sequence,
                **deepcopy(event),
            }

            self._events.append(
                stored_event
            )

            return deepcopy(
                stored_event
            )


    def get_events(
        self,
        after_sequence: int = 0,
    ) -> list[dict]:
        with self._lock:
            return [
                deepcopy(event)
                for event in self._events
                if event["sequence"]
                > after_sequence
            ]


    def clear(self) -> None:
        with self._lock:
            self._events.clear()

            self._sequence = 0


EVENT_MANAGER = EventManager()