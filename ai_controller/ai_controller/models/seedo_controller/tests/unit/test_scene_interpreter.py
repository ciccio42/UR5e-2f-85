from __future__ import annotations

import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from results import (
    RawSceneObject,
    RawSceneState,
    ScenePerceptionResult,
)

from ai_controller.models.seedo_controller import (
    scene_interpreter as scene_interpreter_module,
)
from ai_controller.models.seedo_controller.scene_interpreter import (
    SceneInterpreter,
)


_DEFAULT_ATTRIBUTES = object()


def _raw_object(
    object_id: str,
    *,
    label: str = "green cube",
    category="cube",
    attributes=_DEFAULT_ATTRIBUTES,
    pixel_coordinates=(10, 20),
    confidence=0.9,
) -> RawSceneObject:
    if attributes is _DEFAULT_ATTRIBUTES:
        attributes = {
            "color": "green",
        }

    return RawSceneObject(
        object_id=object_id,
        label=label,
        pixel_coordinates=pixel_coordinates,
        position_camera=(0.1, 0.2, 0.3),
        position_base=(0.4, 0.5, 0.6),
        mask=None,
        confidence=confidence,
        category=category,
        attributes=attributes,
    )


def _perception_result(
    objects,
    overlay_image_path,
) -> ScenePerceptionResult:
    return ScenePerceptionResult(
        raw_scene=RawSceneState(
            objects=tuple(objects),
        ),
        overlay_image_path=overlay_image_path,
    )


def _write_overlay(tmp_path):
    overlay_path = (
        tmp_path
        / "overlay.jpg"
    )

    image = np.zeros(
        (32, 32, 3),
        dtype=np.uint8,
    )

    assert cv2.imwrite(
        str(overlay_path),
        image,
    )

    return overlay_path


def _install_fake_openai(
    monkeypatch,
    raw_output,
    captured,
):
    class FakeCompletions:
        def create(
            self,
            **kwargs,
        ):
            captured.update(kwargs)

            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(
                            content=raw_output,
                        )
                    )
                ]
            )

    class FakeChat:
        def __init__(self):
            self.completions = (
                FakeCompletions()
            )

    class FakeOpenAI:
        def __init__(self):
            self.chat = FakeChat()

    monkeypatch.setattr(
        scene_interpreter_module,
        "OpenAI",
        FakeOpenAI,
    )


@pytest.mark.parametrize(
    ("perception_mode", "expected"),
    [
        ("generalized", "generalized"),
        ("GENERALIZED", "generalized"),
        ("  generalized  ", "generalized"),
        ("prior_guided", "prior_guided"),
        ("PRIOR_GUIDED", "prior_guided"),
        ("  prior_guided  ", "prior_guided"),
    ],
)
def test_constructor_normalizes_perception_mode(
    perception_mode,
    expected,
):
    interpreter = SceneInterpreter(
        perception_mode=perception_mode,
    )

    assert interpreter.perception_mode == expected


@pytest.mark.parametrize(
    "perception_mode",
    [
        "",
        "unknown",
        "general",
        "prior-guided",
    ],
)
def test_constructor_rejects_invalid_perception_mode(
    perception_mode,
):
    with pytest.raises(
        ValueError,
        match="Invalid perception_mode",
    ):
        SceneInterpreter(
            perception_mode=perception_mode,
        )


def test_constructor_stores_model():
    interpreter = SceneInterpreter(
        model="test-model",
    )

    assert interpreter.model == "test-model"


def test_run_rejects_empty_raw_scene(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(),
        overlay_image_path=_write_overlay(
            tmp_path
        ),
    )

    with pytest.raises(
        ValueError,
        match="Raw scene contains no objects",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_run_rejects_missing_overlay(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(
            _raw_object("raw_1"),
        ),
        overlay_image_path=None,
    )

    with pytest.raises(
        ValueError,
        match="does not contain a raw scene overlay",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_run_rejects_nonexistent_overlay(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(
            _raw_object("raw_1"),
        ),
        overlay_image_path=(
            tmp_path
            / "missing.jpg"
        ),
    )

    with pytest.raises(
        FileNotFoundError,
        match="Raw scene overlay does not exist",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_generalized_preserves_raw_object_ids(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    raw_objects = (
        _raw_object(
            "raw_cube",
            label="green cube",
            category="cube",
            attributes={
                "color": "green",
            },
            pixel_coordinates=(100, 200),
        ),
        _raw_object(
            "raw_bin",
            label="storage bin",
            category="bin",
            attributes={
                "type": "container",
            },
            pixel_coordinates=(300, 200),
        ),
    )

    scene_state = interpreter.run(
        perception_result=_perception_result(
            objects=raw_objects,
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
    )

    assert tuple(
        obj.object_id
        for obj in scene_state.objects
    ) == (
        "raw_cube",
        "raw_bin",
    )


def test_generalized_preserves_runtime_semantics(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    raw_object = _raw_object(
        "raw_1",
        label="green cube",
        category="cube",
        attributes={
            "color": "green",
            "shape": "cube",
        },
        pixel_coordinates=(123, 456),
    )

    scene_state = interpreter.run(
        perception_result=_perception_result(
            objects=(
                raw_object,
            ),
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
    )

    scene_object = scene_state.objects[0]

    assert scene_object.object_id == "raw_1"
    assert scene_object.label == "green cube"
    assert scene_object.category == "cube"

    assert scene_object.attributes == {
        "color": "green",
        "shape": "cube",
    }

    assert (
        scene_object.attributes
        is not raw_object.attributes
    )

    assert scene_object.pixel_coordinates == (
        123,
        456,
    )

    assert scene_object.position_camera == (
        0.1,
        0.2,
        0.3,
    )

    assert scene_object.position_base == (
        0.4,
        0.5,
        0.6,
    )


def test_generalized_does_not_call_vlm(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    def fail_if_called(*args, **kwargs):
        raise AssertionError(
            "_assign_semantic_names must not "
            "be called in generalized mode."
        )

    monkeypatch.setattr(
        interpreter,
        "_assign_semantic_names",
        fail_if_called,
    )

    scene_state = interpreter.run(
        perception_result=_perception_result(
            objects=(
                _raw_object("raw_1"),
            ),
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
    )

    assert scene_state.objects[0].object_id == (
        "raw_1"
    )


@pytest.mark.parametrize(
    "category",
    [
        None,
        "",
    ],
)
def test_generalized_rejects_missing_category(
    tmp_path,
    category,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(
            _raw_object(
                "raw_1",
                category=category,
            ),
        ),
        overlay_image_path=_write_overlay(
            tmp_path
        ),
    )

    with pytest.raises(
        RuntimeError,
        match="Missing category for generalized object",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


@pytest.mark.parametrize(
    "attributes",
    [
        None,
        [],
        "green",
        42,
    ],
)
def test_generalized_rejects_invalid_attributes(
    tmp_path,
    attributes,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(
            _raw_object(
                "raw_1",
                attributes=attributes,
            ),
        ),
        overlay_image_path=_write_overlay(
            tmp_path
        ),
    )

    with pytest.raises(
        RuntimeError,
        match="Invalid attributes for generalized object",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_generalized_rejects_duplicate_object_ids(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    perception_result = _perception_result(
        objects=(
            _raw_object("duplicate"),
            _raw_object(
                "duplicate",
                category="bin",
            ),
        ),
        overlay_image_path=_write_overlay(
            tmp_path
        ),
    )

    with pytest.raises(
        RuntimeError,
        match="duplicate object ID",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_generalized_writes_artifacts(
    tmp_path,
):
    interpreter = SceneInterpreter(
        perception_mode="generalized",
    )

    artifacts_dir = (
        tmp_path
        / "nested"
        / "scene_interpreter"
    )

    scene_state = interpreter.run(
        perception_result=_perception_result(
            objects=(
                _raw_object(
                    "raw_cube",
                    label="green cube",
                    category="cube",
                    attributes={
                        "color": "green",
                    },
                    pixel_coordinates=(10, 20),
                ),
                _raw_object(
                    "raw_bin",
                    label="storage bin",
                    category="bin",
                    attributes={},
                    pixel_coordinates=(30, 40),
                ),
            ),
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        ),
        artifacts_dir=artifacts_dir,
    )

    interpretation_path = (
        artifacts_dir
        / "scene_interpretation.json"
    )

    state_path = (
        artifacts_dir
        / "scene_state.json"
    )

    assert interpretation_path.is_file()
    assert state_path.is_file()

    with interpretation_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        interpretation = json.load(
            stream
        )

    assert interpretation == {
        "objects": [
            {
                "raw_object_id": "raw_cube",
                "semantic_name": "raw_cube",
            },
            {
                "raw_object_id": "raw_bin",
                "semantic_name": "raw_bin",
            },
        ]
    }

    with state_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        state = json.load(
            stream
        )

    assert state == {
        "objects": [
            {
                "object_id": "raw_cube",
                "label": "green cube",
                "category": "cube",
                "attributes": {
                    "color": "green",
                },
                "pixel_coordinates": [
                    10,
                    20,
                ],
                "position_camera": [
                    0.1,
                    0.2,
                    0.3,
                ],
                "position_base": [
                    0.4,
                    0.5,
                    0.6,
                ],
            },
            {
                "object_id": "raw_bin",
                "label": "storage bin",
                "category": "bin",
                "attributes": {},
                "pixel_coordinates": [
                    30,
                    40,
                ],
                "position_camera": [
                    0.1,
                    0.2,
                    0.3,
                ],
                "position_base": [
                    0.4,
                    0.5,
                    0.6,
                ],
            },
        ]
    }

    assert len(scene_state.objects) == 2


def test_prior_guided_uses_semantic_names(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    overlay_path = _write_overlay(
        tmp_path
    )

    raw_scene = RawSceneState(
        objects=(
            _raw_object(
                "raw_cube",
                label="red cube",
                category="cube",
                attributes={
                    "color": "red",
                },
            ),
            _raw_object(
                "raw_bin",
                label="storage bin",
                category="bin",
                attributes={},
            ),
        )
    )

    captured = {}

    def fake_assign_semantic_names(
        *,
        raw_scene,
        overlay_image_path,
        artifacts_dir=None,
    ):
        captured[
            "raw_scene"
        ] = raw_scene

        captured[
            "overlay_image_path"
        ] = overlay_image_path

        captured[
            "artifacts_dir"
        ] = artifacts_dir

        return {
            "raw_cube": "red cube",
            "raw_bin": "first storage bin",
        }

    monkeypatch.setattr(
        interpreter,
        "_assign_semantic_names",
        fake_assign_semantic_names,
    )

    scene_state = interpreter.run(
        perception_result=ScenePerceptionResult(
            raw_scene=raw_scene,
            overlay_image_path=overlay_path,
        )
    )

    assert captured["raw_scene"] is raw_scene

    assert (
        captured["overlay_image_path"]
        == overlay_path
    )

    assert captured["artifacts_dir"] is None

    assert tuple(
        obj.object_id
        for obj in scene_state.objects
    ) == (
        "red cube",
        "first storage bin",
    )


def test_prior_guided_does_not_propagate_generalized_metadata(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setattr(
        interpreter,
        "_assign_semantic_names",
        lambda **kwargs: {
            "raw_1": "green cube",
        },
    )

    scene_state = interpreter.run(
        perception_result=_perception_result(
            objects=(
                _raw_object(
                    "raw_1",
                    label="green cube",
                    category="cube",
                    attributes={
                        "color": "green",
                    },
                ),
            ),
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        )
    )

    scene_object = (
        scene_state.objects[0]
    )

    assert scene_object.object_id == (
        "green cube"
    )

    assert scene_object.label == (
        "green cube"
    )

    assert scene_object.category is None
    assert scene_object.attributes == {}


def test_run_rejects_incomplete_semantic_mapping(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setattr(
        interpreter,
        "_assign_semantic_names",
        lambda **kwargs: {
            "raw_1": "green cube",
        },
    )

    perception_result = (
        _perception_result(
            objects=(
                _raw_object("raw_1"),
                _raw_object(
                    "raw_2",
                    category="bin",
                ),
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )
    )

    with pytest.raises(
        RuntimeError,
        match="did not assign exactly one semantic name",
    ):
        interpreter.run(
            perception_result=perception_result,
        )


def test_prior_guided_scene_state_artifact_excludes_category_and_attributes(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setattr(
        interpreter,
        "_assign_semantic_names",
        lambda **kwargs: {
            "raw_1": "green cube",
        },
    )

    artifacts_dir = (
        tmp_path
        / "artifacts"
    )

    interpreter.run(
        perception_result=_perception_result(
            objects=(
                _raw_object(
                    "raw_1",
                    label="green cube",
                    category="cube",
                    attributes={
                        "color": "green",
                    },
                ),
            ),
            overlay_image_path=_write_overlay(
                tmp_path
            ),
        ),
        artifacts_dir=artifacts_dir,
    )

    with (
        artifacts_dir
        / "scene_state.json"
    ).open(
        "r",
        encoding="utf-8",
    ) as stream:
        state = json.load(
            stream
        )

    assert state["objects"][0] == {
        "object_id": "green cube",
        "label": "green cube",
        "pixel_coordinates": [
            10,
            20,
        ],
        "position_camera": [
            0.1,
            0.2,
            0.3,
        ],
        "position_base": [
            0.4,
            0.5,
            0.6,
        ],
    }


def test_prior_guided_vlm_requires_api_key(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.delenv(
        "OPENAI_API_KEY",
        raising=False,
    )

    with pytest.raises(
        ValueError,
        match="OPENAI_API_KEY is not configured",
    ):
        interpreter._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object("raw_1"),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )


def test_prior_guided_vlm_returns_valid_mapping(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        model="test-model",
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_output = json.dumps(
        {
            "objects": [
                {
                    "raw_object_id": "raw_cube",
                    "semantic_name": "red cube",
                },
                {
                    "raw_object_id": "raw_bin",
                    "semantic_name": "first storage bin",
                },
            ]
        }
    )

    captured = {}

    _install_fake_openai(
        monkeypatch,
        raw_output,
        captured,
    )

    artifacts_dir = (
        tmp_path
        / "artifacts"
    )

    artifacts_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    semantic_names = (
        interpreter
        ._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object(
                        "raw_cube",
                        label="red cube",
                    ),
                    _raw_object(
                        "raw_bin",
                        label="storage bin",
                        category="bin",
                        attributes={},
                    ),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
            artifacts_dir=artifacts_dir,
        )
    )

    assert semantic_names == {
        "raw_cube": "red cube",
        "raw_bin": "first storage bin",
    }

    assert captured["model"] == (
        "test-model"
    )

    assert captured["temperature"] == 0
    assert captured["max_tokens"] == 800

    assert (
        captured["messages"][0][
            "content"
        ]
        == scene_interpreter_module.PRIOR_GUIDED_SCENE_INTERPRETER_SYSTEM_PROMPT
    )

    assert captured["response_format"] == {
        "type": "json_schema",
        "json_schema": (
            scene_interpreter_module.SCENE_INTERPRETATION_SCHEMA
        ),
    }

    artifact_path = (
        artifacts_dir
        / "scene_interpretation.json"
    )

    assert artifact_path.is_file()

    with artifact_path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        artifact = json.load(
            stream
        )

    assert artifact == json.loads(
        raw_output
    )


def test_prior_guided_vlm_rejects_empty_output(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    _install_fake_openai(
        monkeypatch,
        None,
        {},
    )

    with pytest.raises(
        RuntimeError,
        match="returned no output",
    ):
        interpreter._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object("raw_1"),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )


def test_prior_guided_vlm_rejects_invalid_object_mapping(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_output = json.dumps(
        {
            "objects": [
                {
                    "raw_object_id": "unknown",
                    "semantic_name": "green cube",
                }
            ]
        }
    )

    _install_fake_openai(
        monkeypatch,
        raw_output,
        {},
    )

    with pytest.raises(
        RuntimeError,
        match="invalid object mapping",
    ):
        interpreter._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object("raw_1"),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )


def test_prior_guided_vlm_cannot_change_authoritative_detector_label(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_output = json.dumps(
        {
            "objects": [
                {
                    "raw_object_id": "raw_cube",
                    "semantic_name": "blue cube",
                }
            ]
        }
    )

    _install_fake_openai(
        monkeypatch,
        raw_output,
        {},
    )

    with pytest.raises(
        RuntimeError,
        match="changed an authoritative semantic detector label",
    ):
        interpreter._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object(
                        "raw_cube",
                        label="red cube",
                    ),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )


def test_prior_guided_vlm_allows_storage_bin_semantic_renaming(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_output = json.dumps(
        {
            "objects": [
                {
                    "raw_object_id": "raw_bin",
                    "semantic_name": "second storage bin",
                }
            ]
        }
    )

    _install_fake_openai(
        monkeypatch,
        raw_output,
        {},
    )

    semantic_names = (
        interpreter
        ._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object(
                        "raw_bin",
                        label="storage bin",
                        category="bin",
                        attributes={},
                    ),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )
    )

    assert semantic_names == {
        "raw_bin": "second storage bin",
    }


def test_prior_guided_vlm_rejects_duplicate_semantic_names(
    tmp_path,
    monkeypatch,
):
    interpreter = SceneInterpreter(
        perception_mode="prior_guided",
    )

    monkeypatch.setenv(
        "OPENAI_API_KEY",
        "test-key",
    )

    raw_output = json.dumps(
        {
            "objects": [
                {
                    "raw_object_id": "raw_cube",
                    "semantic_name": "shared name",
                },
                {
                    "raw_object_id": "raw_bin",
                    "semantic_name": "shared name",
                },
            ]
        }
    )

    _install_fake_openai(
        monkeypatch,
        raw_output,
        {},
    )

    with pytest.raises(
        RuntimeError,
        match="duplicate semantic names",
    ):
        interpreter._assign_semantic_names(
            raw_scene=RawSceneState(
                objects=(
                    _raw_object(
                        "raw_cube",
                        label="shared name",
                    ),
                    _raw_object(
                        "raw_bin",
                        label="storage bin",
                        category="bin",
                        attributes={},
                    ),
                )
            ),
            overlay_image_path=(
                _write_overlay(
                    tmp_path
                )
            ),
        )