from __future__ import annotations

import contextvars
import json
import os
import threading

from contextlib import contextmanager
from pathlib import Path
from typing import Any


_CURRENT_STAGE = contextvars.ContextVar(
    "seedo_token_usage_stage",
    default=("unattributed", "unattributed"),
)


# Prices in USD per 1M tokens.
#
# Pricing reference checked on 2026-10-23:
# https://developers.openai.com/api/docs/models/gpt-4o
PRICING_REFERENCE = {
    "currency": "USD",
    "unit": "per_1m_tokens",
    "checked_on": "2026-10-23",
    "source": (
        "https://developers.openai.com/api/docs/models/gpt-4o"
    ),
    "models": {
        "gpt-4o": {
            "input": 2.50,
            "cached_input": 1.25,
            "output": 10.00,
        },
        "gpt-4o-2024-08-06": {
            "input": 2.50,
            "cached_input": 1.25,
            "output": 10.00,
        },
        "gpt-4o-2024-11-20": {
            "input": 2.50,
            "cached_input": 1.25,
            "output": 10.00,
        },
    },
}


EXPECTED_MODULES = {
    "demonstration": (
        "keyframe_selector",
        "visual_prompter",
        "action_planner",
    ),
    "runtime": (
        "scene_perceiver",
        "scene_interpreter",
        "lmp_generator",
    ),
}


def _read_field(
    obj: Any,
    name: str,
    default: Any = None,
) -> Any:

    if obj is None:
        return default

    if isinstance(obj, dict):
        return obj.get(
            name,
            default,
        )

    return getattr(
        obj,
        name,
        default,
    )


def _as_int(
    value: Any,
) -> int:

    try:
        return int(
            value or 0
        )

    except (
        TypeError,
        ValueError,
    ):
        return 0


class TokenUsageRecorder:

    def __init__(
        self,
    ) -> None:

        self._lock = threading.Lock()

        self.hooked_apis: list[str] = []
        self.hook_warnings: list[str] = []

        self.reset()

    def reset(
        self,
    ) -> None:

        with self._lock:

            self.calls: list[
                dict[str, Any]
            ] = []

            self.warnings: list[str] = []

            self._sequence = 0

    @contextmanager
    def stage(
        self,
        phase: str,
        module: str,
    ):

        token = _CURRENT_STAGE.set(
            (
                str(phase),
                str(module),
            )
        )

        try:
            yield

        finally:
            _CURRENT_STAGE.reset(
                token
            )

    def add_hooked_api(
        self,
        api_name: str,
    ) -> None:

        with self._lock:

            if api_name not in self.hooked_apis:

                self.hooked_apis.append(
                    api_name
                )

    def add_hook_warning(
        self,
        message: str,
    ) -> None:

        with self._lock:

            if message not in self.hook_warnings:

                self.hook_warnings.append(
                    message
                )

    def add_warning(
        self,
        message: str,
    ) -> None:

        with self._lock:

            self.warnings.append(
                str(message)
            )

    def record_response(
        self,
        response: Any,
        *,
        api: str,
        requested_model: str | None,
    ) -> None:

        (
            phase,
            module,
        ) = _CURRENT_STAGE.get()

        usage = _read_field(
            response,
            "usage",
        )

        response_model = _read_field(
            response,
            "model",
        )

        response_id = _read_field(
            response,
            "id",
        )

        model = str(
            response_model
            or requested_model
            or "unknown"
        )

        if usage is None:

            input_tokens = 0
            cached_input_tokens = 0
            uncached_input_tokens = 0
            output_tokens = 0
            reasoning_tokens = 0
            total_tokens = 0
            estimated_cost_usd = None

            usage_available = False

        else:

            usage_available = True

            if api == "responses":

                input_tokens = _as_int(
                    _read_field(
                        usage,
                        "input_tokens",
                    )
                )

                output_tokens = _as_int(
                    _read_field(
                        usage,
                        "output_tokens",
                    )
                )

                total_tokens = _as_int(
                    _read_field(
                        usage,
                        "total_tokens",
                    )
                )

                input_details = _read_field(
                    usage,
                    "input_tokens_details",
                )

                output_details = _read_field(
                    usage,
                    "output_tokens_details",
                )

            else:

                input_tokens = _as_int(
                    _read_field(
                        usage,
                        "prompt_tokens",
                    )
                )

                output_tokens = _as_int(
                    _read_field(
                        usage,
                        "completion_tokens",
                    )
                )

                total_tokens = _as_int(
                    _read_field(
                        usage,
                        "total_tokens",
                    )
                )

                input_details = _read_field(
                    usage,
                    "prompt_tokens_details",
                )

                output_details = _read_field(
                    usage,
                    "completion_tokens_details",
                )

            cached_input_tokens = _as_int(
                _read_field(
                    input_details,
                    "cached_tokens",
                )
            )

            reasoning_tokens = _as_int(
                _read_field(
                    output_details,
                    "reasoning_tokens",
                )
            )

            uncached_input_tokens = max(
                input_tokens
                - cached_input_tokens,
                0,
            )

            if total_tokens <= 0:

                total_tokens = (
                    input_tokens
                    + output_tokens
                )

            estimated_cost_usd = (
                self._estimate_cost(
                    model=model,
                    uncached_input_tokens=(
                        uncached_input_tokens
                    ),
                    cached_input_tokens=(
                        cached_input_tokens
                    ),
                    output_tokens=(
                        output_tokens
                    ),
                )
            )

        call = {
            "sequence": None,
            "phase": phase,
            "module": module,
            "api": api,
            "model": model,
            "response_id": response_id,
            "usage_available": (
                usage_available
            ),
            "input_tokens": (
                input_tokens
            ),
            "cached_input_tokens": (
                cached_input_tokens
            ),
            "uncached_input_tokens": (
                uncached_input_tokens
            ),
            "output_tokens": (
                output_tokens
            ),
            "reasoning_tokens": (
                reasoning_tokens
            ),
            "total_tokens": (
                total_tokens
            ),
            "estimated_cost_usd": (
                estimated_cost_usd
            ),
        }

        with self._lock:

            self._sequence += 1

            call[
                "sequence"
            ] = self._sequence

            self.calls.append(
                call
            )

            if phase == "unattributed":

                self.warnings.append(
                    "OpenAI call captured outside "
                    "a token-usage stage: "
                    f"model={model}, api={api}."
                )

            if not usage_available:

                self.warnings.append(
                    "OpenAI response did not expose "
                    "token usage: "
                    f"model={model}, api={api}."
                )

            elif estimated_cost_usd is None:

                self.warnings.append(
                    "No pricing entry for model "
                    f"{model!r}; token counts are "
                    "valid but cost is incomplete."
                )

        print(
            "[TOKEN USAGE] "
            f"{phase}/{module} | "
            f"model={model} | "
            f"input={input_tokens} | "
            f"cached={cached_input_tokens} | "
            f"output={output_tokens} | "
            f"total={total_tokens} | "
            f"cost_usd={estimated_cost_usd}",
            flush=True,
        )

    @staticmethod
    def _estimate_cost(
        *,
        model: str,
        uncached_input_tokens: int,
        cached_input_tokens: int,
        output_tokens: int,
    ) -> float | None:

        rates = (
            PRICING_REFERENCE[
                "models"
            ].get(
                model
            )
        )

        if rates is None:
            return None

        cost = (
            uncached_input_tokens
            * float(
                rates["input"]
            )
            + cached_input_tokens
            * float(
                rates[
                    "cached_input"
                ]
            )
            + output_tokens
            * float(
                rates["output"]
            )
        ) / 1_000_000.0

        return round(
            cost,
            10,
        )

    @staticmethod
    def _empty_summary(
    ) -> dict[str, Any]:

        return {
            "api_calls": 0,
            "input_tokens": 0,
            "cached_input_tokens": 0,
            "uncached_input_tokens": 0,
            "output_tokens": 0,
            "reasoning_tokens": 0,
            "total_tokens": 0,
            "estimated_cost_usd": 0.0,
            "cost_complete": True,
        }

    @classmethod
    def _accumulate(
        cls,
        summary: dict[str, Any],
        call: dict[str, Any],
    ) -> None:

        summary[
            "api_calls"
        ] += 1

        for key in (
            "input_tokens",
            "cached_input_tokens",
            "uncached_input_tokens",
            "output_tokens",
            "reasoning_tokens",
            "total_tokens",
        ):

            summary[key] += int(
                call[key]
            )

        call_cost = (
            call[
                "estimated_cost_usd"
            ]
        )

        if (
            not call[
                "usage_available"
            ]
            or call_cost is None
        ):

            summary[
                "cost_complete"
            ] = False

        else:

            summary[
                "estimated_cost_usd"
            ] += float(
                call_cost
            )

    @staticmethod
    def _finalize_summary(
        summary: dict[str, Any],
    ) -> dict[str, Any]:

        summary = dict(
            summary
        )

        if summary[
            "cost_complete"
        ]:

            summary[
                "estimated_cost_usd"
            ] = round(
                float(
                    summary[
                        "estimated_cost_usd"
                    ]
                ),
                10,
            )

        else:

            summary[
                "known_cost_usd"
            ] = round(
                float(
                    summary[
                        "estimated_cost_usd"
                    ]
                ),
                10,
            )

            summary[
                "estimated_cost_usd"
            ] = None

        return summary

    def snapshot(
        self,
    ) -> dict[str, Any]:

        with self._lock:

            calls = [
                dict(call)
                for call
                in self.calls
            ]

            warnings = list(
                self.warnings
            )

            hook_warnings = list(
                self.hook_warnings
            )

            hooked_apis = list(
                self.hooked_apis
            )

        modules = {}

        for (
            phase,
            names,
        ) in EXPECTED_MODULES.items():

            for name in names:

                modules[
                    name
                ] = self._empty_summary()

                modules[
                    name
                ][
                    "phase"
                ] = phase

        subtotals = {
            "demonstration": (
                self._empty_summary()
            ),
            "runtime": (
                self._empty_summary()
            ),
            "unattributed": (
                self._empty_summary()
            ),
        }

        total = (
            self._empty_summary()
        )

        for call in calls:

            module = call[
                "module"
            ]

            phase = call[
                "phase"
            ]

            if module not in modules:

                modules[
                    module
                ] = self._empty_summary()

                modules[
                    module
                ][
                    "phase"
                ] = phase

            if phase not in subtotals:

                subtotals[
                    phase
                ] = self._empty_summary()

            self._accumulate(
                modules[module],
                call,
            )

            self._accumulate(
                subtotals[phase],
                call,
            )

            self._accumulate(
                total,
                call,
            )

        modules = {
            name: (
                self._finalize_summary(
                    summary
                )
            )
            for (
                name,
                summary,
            )
            in modules.items()
        }

        subtotals = {
            phase: (
                self._finalize_summary(
                    summary
                )
            )
            for (
                phase,
                summary,
            )
            in subtotals.items()
        }

        total = (
            self._finalize_summary(
                total
            )
        )

        cost_summary = {
            "currency": PRICING_REFERENCE[
                "currency"
            ],
            "demo_phase_usd": (
                subtotals[
                    "demonstration"
                ][
                    "estimated_cost_usd"
                ]
            ),
            "runtime_phase_usd": (
                subtotals[
                    "runtime"
                ][
                    "estimated_cost_usd"
                ]
            ),
            "total_usd": (
                total[
                    "estimated_cost_usd"
                ]
            ),
        }

        return {
            "pricing_reference": (
                PRICING_REFERENCE
            ),
            "hooked_apis": (
                hooked_apis
            ),
            "modules": modules,
            "subtotals": subtotals,
            "total": total,
            "calls": calls,
            "warnings": (
                hook_warnings
                + warnings
            ),
            "cost_summary": (
                cost_summary
            ),
        }

    def save(
        self,
        path: str | Path,
    ) -> None:

        output_path = Path(
            path
        )

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        with output_path.open(
            "w",
            encoding="utf-8",
        ) as stream:

            json.dump(
                self.snapshot(),
                stream,
                indent=2,
                ensure_ascii=False,
            )


TOKEN_USAGE = TokenUsageRecorder()


_HOOK_INSTALL_LOCK = (
    threading.Lock()
)

_HOOKS_INSTALLED = False


def _install_resource_hook(
    resource: Any,
    *,
    api_name: str,
) -> None:

    resource_class = type(
        resource
    )

    original_create = getattr(
        resource_class,
        "create",
    )

    if getattr(
        original_create,
        "_seedo_token_usage_hook",
        False,
    ):

        TOKEN_USAGE.add_hooked_api(
            api_name
        )

        return

    def tracked_create(
        resource_self,
        *args,
        **kwargs,
    ):

        response = original_create(
            resource_self,
            *args,
            **kwargs,
        )

        if bool(
            kwargs.get(
                "stream",
                False,
            )
        ):

            TOKEN_USAGE.add_warning(
                "Streaming OpenAI call was not "
                "token-accounted by the synchronous "
                f"hook: api={api_name}."
            )

            return response

        TOKEN_USAGE.record_response(
            response,
            api=api_name,
            requested_model=(
                kwargs.get(
                    "model"
                )
            ),
        )

        return response

    tracked_create._seedo_token_usage_hook = (
        True
    )

    tracked_create._seedo_original_create = (
        original_create
    )

    setattr(
        resource_class,
        "create",
        tracked_create,
    )

    TOKEN_USAGE.add_hooked_api(
        api_name
    )


def install_openai_usage_hooks(
) -> None:

    global _HOOKS_INSTALLED

    with _HOOK_INSTALL_LOCK:

        if _HOOKS_INSTALLED:
            return

        try:

            import openai

            # No API request is performed here.
            # The temporary client is used only to obtain
            # the SDK resource classes that need hooking.
            probe_client = openai.OpenAI(
                api_key=(
                    os.environ.get(
                        "OPENAI_API_KEY"
                    )
                    or (
                        "token-usage-"
                        "instrumentation"
                    )
                )
            )

            _install_resource_hook(
                probe_client.chat.completions,
                api_name=(
                    "chat_completions"
                ),
            )

            responses_resource = getattr(
                probe_client,
                "responses",
                None,
            )

            if responses_resource is not None:

                _install_resource_hook(
                    responses_resource,
                    api_name="responses",
                )

            _HOOKS_INSTALLED = True

            print(
                "[TOKEN USAGE] "
                "OpenAI usage hooks installed: "
                + ", ".join(
                    TOKEN_USAGE.hooked_apis
                ),
                flush=True,
            )

        except Exception as exc:

            TOKEN_USAGE.add_hook_warning(
                "Could not install OpenAI "
                "usage hooks: "
                f"{type(exc).__name__}: "
                f"{exc}"
            )

            print(
                "[TOKEN USAGE] WARNING: "
                "could not install OpenAI "
                f"usage hooks: {exc}",
                flush=True,
            )