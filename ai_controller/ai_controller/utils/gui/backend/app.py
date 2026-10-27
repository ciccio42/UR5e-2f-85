from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from pathlib import Path

from fastapi.responses import (
    FileResponse,
    StreamingResponse,
)
from fastapi.staticfiles import StaticFiles

from ai_controller.utils.gui.backend.controllers import list_controllers
from ai_controller.utils.gui.backend.environment import (
    detect_execution_mode,
    ros_available,
)
from ai_controller.utils.gui.backend.experiment import (
    get_experiment_options,
)
from ai_controller.utils.gui.backend.interaction_manager import (
    INTERACTION_MANAGER,
)
from ai_controller.utils.gui.backend.process_manager import (
    PROCESS_MANAGER,
)
from ai_controller.utils.gui.backend.experiment_runner import (
    ExperimentRequest,
    build_experiment_command,
)
from ai_controller.utils.gui.backend.event_manager import (
    EVENT_MANAGER,
)
from ai_controller.utils.gui.backend.camera_stream import (
    FRONT_CAMERA_STREAM,
)
from ai_controller.utils.gui.backend.artifact_manager import (
    ARTIFACT_MANAGER,
)

app = FastAPI(
    title="SeeDo Experiment Dashboard",
    version="0.1.0",
)

GUI_ROOT = Path(__file__).resolve().parent.parent
FRONTEND_ROOT = GUI_ROOT / "frontend"
ACTIVE_EXPERIMENT: dict | None = None

app.mount(
    "/static",
    StaticFiles(
        directory=FRONTEND_ROOT,
    ),
    name="static",
)

@app.get("/")
def dashboard():
    return FileResponse(
        FRONTEND_ROOT / "index.html"
    )

class InteractionResponse(BaseModel):
    value: str


@app.get("/api/health")
def health():
    execution_mode, detection_source = detect_execution_mode()

    return {
        "status": "ok",
        "controller": "seedo",
        "execution_mode": execution_mode,
        "execution_mode_source": detection_source,
        "ros_available": ros_available(),
    }


@app.get("/api/controllers")
def controllers():
    return {
        "controllers": list_controllers(),
    }


@app.get("/api/experiment/options")
def experiment_options():
    return get_experiment_options()


@app.get("/api/interactions/pending")
def pending_interaction():
    return {
        "interaction": INTERACTION_MANAGER.get_pending(),
    }


@app.post("/api/interactions/{interaction_id}/respond")
def respond_to_interaction(
    interaction_id: str,
    response: InteractionResponse,
):
    try:
        INTERACTION_MANAGER.respond(
            interaction_id=interaction_id,
            value=response.value,
        )

    except (LookupError, RuntimeError) as exc:
        return {
            "status": "error",
            "message": str(exc),
        }

    return {
        "status": "ok",
        "interaction_id": interaction_id,
    }


@app.get("/api/process/status")
def process_status():
    return PROCESS_MANAGER.get_status()


@app.get("/api/process/logs")
def process_logs(
    after_sequence: int = 0,
):
    return {
        "logs": PROCESS_MANAGER.get_logs(
            after_sequence=after_sequence,
        ),
    }

@app.get("/api/experiment/demo-video")
def experiment_demo_video():

    if ACTIVE_EXPERIMENT is None:
        raise HTTPException(
            status_code=404,
            detail="No active experiment.",
        )

    demo_path_string = ACTIVE_EXPERIMENT.get(
        "demo_path"
    )

    if not demo_path_string:
        raise HTTPException(
            status_code=404,
            detail="No demonstration video is associated "
            "with the active experiment.",
        )

    demo_path = Path(
        demo_path_string
    ).resolve()

    if not demo_path.is_file():
        raise HTTPException(
            status_code=404,
            detail=f"Demonstration video not found: {demo_path}",
        )

    return FileResponse(
        path=demo_path,
        media_type="video/mp4",
    )
    
@app.post("/api/process/stop")
def stop_process():
    PROCESS_MANAGER.stop()

    return {
        "status": "ok",
    }

@app.post("/api/experiment/preview")
def preview_experiment(
    request: ExperimentRequest,
):
    try:
        return build_experiment_command(
            request
        )

    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail=str(exc),
        ) from exc

@app.post("/api/experiment/start")
def start_experiment(
    request: ExperimentRequest,
):
    global ACTIVE_EXPERIMENT

    try:
        experiment = build_experiment_command(
            request
        )

        PROCESS_MANAGER.start(
            command=experiment["command"],
            cwd="/home/ros2_ws/src/UR5e-2f-85",
        )

        ACTIVE_EXPERIMENT = experiment

    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail=str(exc),
        ) from exc

    except RuntimeError as exc:
        raise HTTPException(
            status_code=409,
            detail=str(exc),
        ) from exc

    return {
        "status": "started",
        "experiment": experiment,
        "process": PROCESS_MANAGER.get_status(),
    }

@app.get("/api/pipeline/events")
def pipeline_events(
    after_sequence: int = 0,
):
    return {
        "events": EVENT_MANAGER.get_events(
            after_sequence=after_sequence,
        ),
    }

class PipelineTestEvent(BaseModel):
    event: str
    stage: str | None = None
    artifact_type: str | None = None
    path: str | None = None


@app.post("/api/pipeline/test-event")
def pipeline_test_event(
    payload: PipelineTestEvent,
):
    return EVENT_MANAGER.register(
        payload.model_dump(
            exclude_none=True
        )
    )

@app.get("/api/camera/front/stream")
def front_camera_stream():

    return StreamingResponse(
        FRONT_CAMERA_STREAM.mjpeg_frames(),
        media_type=(
            "multipart/x-mixed-replace; "
            "boundary=frame"
        ),
        headers={
            "Cache-Control": "no-store",
        },
    )

@app.on_event("shutdown")
def shutdown_front_camera_stream():

    FRONT_CAMERA_STREAM.stop()

@app.get("/api/artifacts/tree")
def artifact_tree():

    return ARTIFACT_MANAGER.tree()

@app.get("/api/artifacts/preview")
def artifact_preview(
    path: str,
):
    try:
        return ARTIFACT_MANAGER.preview(
            path
        )

    except (
        RuntimeError,
        ValueError,
        FileNotFoundError,
    ) as exc:
        raise HTTPException(
            status_code=404,
            detail=str(exc),
        ) from exc

@app.get("/api/artifacts/file")
def artifact_file(
    path: str,
    download: bool = False,
):

    try:
        artifact_path = (
            ARTIFACT_MANAGER.resolve_file(
                path
            )
        )

        media_type = (
            ARTIFACT_MANAGER.media_type(
                path
            )
        )

    except (
        RuntimeError,
        ValueError,
        FileNotFoundError,
    ) as exc:
        raise HTTPException(
            status_code=404,
            detail=str(exc),
        ) from exc


    if download:

        return FileResponse(
            path=artifact_path,
            media_type=media_type,
            filename=artifact_path.name,
        )


    return FileResponse(
        path=artifact_path,
        media_type=media_type,
    )