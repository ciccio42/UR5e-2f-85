const elements = {
    systemStatus: document.getElementById("system-status"),
    statusDot: document.getElementById("status-dot"),

    controllerSelect: document.getElementById("controller-select"),
    executionTarget: document.getElementById("execution-target"),
    taskSelect: document.getElementById("task-select"),
    executionTypeSelect: document.getElementById("execution-type-select"),

    demoTask: document.getElementById("demo-task"),
    demoTaskHint: document.getElementById("demo-task-hint"),
    demoPath: document.getElementById("demo-path"),

    isaacPanel: document.getElementById("isaac-panel"),
    configureScene: document.getElementById("configure-scene"),
    isaacSceneFields: document.getElementById("isaac-scene-fields"),
    sceneTask: document.getElementById("scene-task"),
    sceneTaskHint: document.getElementById("scene-task-hint"),
    sceneTrajectory: document.getElementById("scene-trajectory"),
    sceneTrajectoryHint: document.getElementById("scene-trajectory-hint"),

    precomputedPanel: document.getElementById("precomputed-panel"),
    precomputedPlan: document.getElementById("precomputed-plan"),
    precomputedDemoScene: document.getElementById("precomputed-demo-scene"),

    configPath: document.getElementById("config-path"),

    previewStatus: document.getElementById("preview-status"),
    previewTarget: document.getElementById("preview-target"),
    previewTask: document.getElementById("preview-task"),
    previewExecution: document.getElementById("preview-execution"),
    previewPerception: document.getElementById("preview-perception"),
    previewCommand: document.getElementById("preview-command"),
    previewError: document.getElementById("preview-error"),

    pipelineTelemetryStatus: document.getElementById("pipeline-telemetry-status"),

    processStatus: document.getElementById("process-status"),
    runtimeIdle: document.getElementById("runtime-idle"),
    runtimeActive: document.getElementById("runtime-active"),
    runtimePid: document.getElementById("runtime-pid"),
    runtimeLiveIndicator: document.getElementById("runtime-live-indicator"),
    terminalStreamIndicator: document.getElementById("terminal-stream-indicator"),
    runtimeConsole: document.getElementById("runtime-console"),

    runtimeVisualButton: document.getElementById("runtime-visual-button"),
    runtimeTerminalButton: document.getElementById("runtime-terminal-button"),
    runtimeViewCaption: document.getElementById("runtime-view-caption"),
    runtimeVisualView: document.getElementById("runtime-visual-view"),
    runtimeTerminalView: document.getElementById("runtime-terminal-view"),

    visualWorkspace: document.getElementById("visual-workspace"),

    demoVideo: document.getElementById("demo-video"),
    demoVideoState: document.getElementById("demo-video-state"),

    actionPlanCard: document.getElementById("action-plan-card"),
    actionPlanState: document.getElementById("action-plan-state"),
    actionPlanText: document.getElementById("action-plan-text"),

    currentStageCard: document.getElementById("current-stage-card"),
    currentStageIndex: document.getElementById("current-stage-index"),
    currentStageKicker: document.getElementById("current-stage-kicker"),
    currentStageTitle: document.getElementById("current-stage-title"),
    currentStageStatus: document.getElementById("current-stage-status"),
    currentStageBadge: document.getElementById("current-stage-badge"),
    currentStageAnimation: document.getElementById("current-stage-animation"),
    currentStageCore: document.getElementById("current-stage-core"),

    robotCameraPanel: document.getElementById("robot-camera-panel"),
    robotCameraState: document.getElementById("robot-camera-state"),
    robotCameraStream: document.getElementById("robot-camera-stream"),

    visualErrorCard: document.getElementById("visual-error-card"),
    visualErrorMessage: document.getElementById("visual-error-message"),
    visualErrorReturnCode: document.getElementById("visual-error-return-code"),
    visualErrorTerminal: document.getElementById("visual-error-terminal"),

    executionErrorOverlay: document.getElementById("execution-error-overlay"),
    executionErrorTitle: document.getElementById("execution-error-title"),
    executionErrorMessage: document.getElementById("execution-error-message"),
    executionErrorReturnCode: document.getElementById("execution-error-return-code"),
    executionErrorTerminal: document.getElementById("execution-error-terminal"),
    executionErrorClose: document.getElementById("execution-error-close"),

    startButton: document.getElementById("start-button"),
    startButtonLabel: document.getElementById("start-button-label"),
    stopButton: document.getElementById("stop-button"),
    launchNote: document.getElementById("launch-note"),

    interactionOverlay: document.getElementById("interaction-overlay"),
    interactionPrompt: document.getElementById("interaction-prompt"),
    interactionValue: document.getElementById("interaction-value"),
    interactionResponseField: document.getElementById("interaction-response-field"),
    interactionSubmit: document.getElementById("interaction-submit"),
    interactionError: document.getElementById("interaction-error"),
};


const dashboardState = {
    options: null,
    preview: null,
    previewTimer: null,

    processStatus: "idle",

    lastLogSequence: 0,
    lastPipelineSequence: 0,

    currentInteractionId: null,
    currentInteractionType: "text",

    pollTimer: null,
    runtimeView: "visual",

    activeExperiment: null,
    lastErrorSignature: null,

    visualEventQueue: [],
    visualEventProcessing: false,
    visualQueueGeneration: 0,
    activeVisualStage: null,

    cameraStreamActive: false,
};


/* ============================================================
   BASIC UI
   ============================================================ */


function populateSelect(select, values, selectedId) {
    select.innerHTML = "";

    for (const value of values) {
        const option = document.createElement("option");

        option.value = value.id;
        option.textContent = value.label;

        if (value.id === selectedId) {
            option.selected = true;
        }

        select.appendChild(option);
    }
}


function formatExecutionMode(mode) {
    if (mode === "isaac") {
        return "Isaac Sim";
    }

    if (mode === "real") {
        return "Real Robot";
    }

    return mode;
}


function setConnectionState(online, message) {
    elements.systemStatus.textContent = message;

    elements.statusDot.classList.remove(
        "loading",
        "online",
        "offline",
    );

    elements.statusDot.classList.add(
        online
            ? "online"
            : "offline"
    );
}


function setPanelVisible(element, visible) {
    element.classList.toggle(
        "visible",
        visible,
    );
}


/* ============================================================
   SETUP
   ============================================================ */


function getSelectedTaskDefinition() {
    if (!dashboardState.options) {
        return null;
    }

    return dashboardState.options.tasks.find(
        task => task.id === elements.taskSelect.value
    );
}


function updateTaskDependentFields() {
    const task = getSelectedTaskDefinition();

    if (!task) {
        return;
    }

    const demo = task.demo_task;

    elements.demoTask.min = demo.minimum;
    elements.demoTask.max = demo.maximum;

    const demoValue = Number(
        elements.demoTask.value
    );

    if (
        !Number.isFinite(demoValue)
        || demoValue < demo.minimum
        || demoValue > demo.maximum
    ) {
        elements.demoTask.value = demo.default;
    }

    elements.demoTaskHint.textContent =
        `Allowed range: ${demo.minimum} – ${demo.maximum}`;

    const scene = task.isaac_scene;

    if (!scene) {
        return;
    }

    elements.sceneTask.min = scene.task.minimum;
    elements.sceneTask.max = scene.task.maximum;
    elements.sceneTask.value = scene.task.default;

    elements.sceneTaskHint.textContent =
        `Allowed range: ${scene.task.minimum} – ${scene.task.maximum}`;

    elements.sceneTrajectory.min = scene.trajectory.minimum;
    elements.sceneTrajectory.max = scene.trajectory.maximum;
    elements.sceneTrajectory.value = scene.trajectory.default;

    if (
        scene.trajectory.minimum
        === scene.trajectory.maximum
    ) {
        elements.sceneTrajectory.disabled = true;

        elements.sceneTrajectoryHint.textContent =
            `Fixed value: ${scene.trajectory.default}`;
    }

    else {
        elements.sceneTrajectory.disabled = false;

        elements.sceneTrajectoryHint.textContent =
            `Allowed range: ${scene.trajectory.minimum} – ${scene.trajectory.maximum}`;
    }
}


function updateIsaacVisibility() {
    if (!dashboardState.options) {
        return;
    }

    const isIsaac =
        dashboardState.options
            .environment
            .execution_mode
        === "isaac";

    setPanelVisible(
        elements.isaacPanel,
        isIsaac,
    );

    setPanelVisible(
        elements.isaacSceneFields,
        (
            isIsaac
            && elements.configureScene.checked
        ),
    );
}


function updateExecutionTypeVisibility() {
    const precomputed =
        elements.executionTypeSelect.value
        === "precomputed";

    setPanelVisible(
        elements.precomputedPanel,
        precomputed,
    );
}


function buildRequest() {
    const demoPath =
        elements.demoPath.value.trim();

    const precomputedPlan =
        elements.precomputedPlan.value.trim();

    const precomputedDemoScene =
        elements.precomputedDemoScene
            .value
            .trim();

    return {
        controller:
            elements.controllerSelect.value,

        task:
            elements.taskSelect.value,

        demo_task:
            Number(
                elements.demoTask.value
            ),

        demo_path:
            demoPath || null,

        execution_type:
            elements.executionTypeSelect.value,

        configure_isaac_scene:
            elements.configureScene.checked,

        scene_task:
            elements.sceneTask.value === ""
                ? null
                : Number(
                    elements.sceneTask.value
                ),

        scene_trajectory:
            elements.sceneTrajectory.value === ""
                ? null
                : Number(
                    elements.sceneTrajectory.value
                ),

        precomputed_plan:
            precomputedPlan || null,

        precomputed_demo_scene:
            precomputedDemoScene || null,
    };
}


/* ============================================================
   PREVIEW
   ============================================================ */


function processIsBusy() {
    return [
        "starting",
        "running",
        "stopping",
    ].includes(
        dashboardState.processStatus
    );
}


function updateStartAvailability() {
    const valid =
        dashboardState.preview !== null;

    const busy =
        processIsBusy();

    elements.startButton.disabled =
        !valid || busy;

    elements.stopButton.classList.toggle(
        "hidden",
        !busy,
    );

    if (busy) {
        elements.launchNote.textContent =
            "An experiment is currently active.";

        return;
    }

    if (valid) {
        elements.launchNote.textContent =
            "Configuration valid. Ready to launch.";

        return;
    }

    elements.launchNote.textContent =
        "Fix the configuration before launching.";
}


function setPreviewLoading() {
    dashboardState.preview =
        null;

    elements.previewStatus.textContent =
        "VALIDATING";

    elements.previewStatus.className =
        "preview-badge loading";

    elements.previewError.classList.remove(
        "visible"
    );

    updateStartAvailability();
}


function setPreviewError(message) {
    dashboardState.preview =
        null;

    elements.previewStatus.textContent =
        "INVALID";

    elements.previewStatus.className =
        "preview-badge invalid";

    elements.previewError.textContent =
        message;

    elements.previewError.classList.add(
        "visible"
    );

    elements.previewCommand.textContent =
        "Configuration is not valid.";

    updateStartAvailability();
}


function renderPreview(preview) {
    dashboardState.preview =
        preview;

    applyPipelineMode(
        preview.perception_mode
    );

    elements.previewStatus.textContent =
        "READY";

    elements.previewStatus.className =
        "preview-badge valid";

    elements.previewError.classList.remove(
        "visible"
    );

    elements.previewTarget.textContent =
        formatExecutionMode(
            preview.execution_mode
        );

    elements.previewTask.textContent =
        preview.task;

    elements.previewExecution.textContent =
        preview.execution_type;

    elements.previewPerception.textContent =
        preview.perception_mode;

    elements.previewCommand.textContent =
        preview.shell_command;

    updateStartAvailability();
}


async function refreshPreview() {
    if (!dashboardState.options) {
        return;
    }

    setPreviewLoading();

    try {
        const response =
            await fetch(
                "/api/experiment/preview",
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "application/json",
                    },

                    body:
                        JSON.stringify(
                            buildRequest()
                        ),
                }
            );

        const payload =
            await response.json();

        if (!response.ok) {
            throw new Error(
                payload.detail
                ?? "Invalid configuration."
            );
        }

        renderPreview(
            payload
        );
    }

    catch (error) {
        setPreviewError(
            error.message
        );
    }
}


function schedulePreview() {
    clearTimeout(
        dashboardState.previewTimer
    );

    dashboardState.previewTimer =
        setTimeout(
            refreshPreview,
            250,
        );
}


/* ============================================================
   PIPELINE MONITOR
   ============================================================ */


function getPipelineStage(stageName) {
    return document.querySelector(
        `.pipeline-stage[data-stage="${stageName}"]`
    );
}


function applyPipelineMode(perceptionMode) {
    const generalized =
        perceptionMode === "generalized";

    document
        .querySelectorAll(
            '.pipeline-stage[data-generalized-only="true"]'
        )
        .forEach(
            stage => {
                const status =
                    stage.querySelector(
                        ".stage-text span"
                    );

                if (generalized) {
                    stage.classList.remove(
                        "skipped"
                    );

                    if (
                        !stage.classList.contains(
                            "running"
                        )
                        &&
                        !stage.classList.contains(
                            "completed"
                        )
                        &&
                        !stage.classList.contains(
                            "failed"
                        )
                    ) {
                        stage.classList.add(
                            "waiting"
                        );

                        if (status) {
                            status.textContent =
                                "Waiting";
                        }
                    }
                }

                else {
                    stage.classList.remove(
                        "waiting",
                        "running",
                        "completed",
                        "failed",
                        "artifact-pulse",
                    );

                    stage.classList.add(
                        "skipped"
                    );

                    if (status) {
                        status.textContent =
                            "Skipped";
                    }
                }
            }
        );

    document
        .querySelectorAll(
            '.pipeline-link[data-generalized-only="true"]'
        )
        .forEach(
            link => {
                link.classList.toggle(
                    "skipped",
                    !generalized,
                );

                if (!generalized) {
                    link.classList.remove(
                        "active",
                        "completed",
                    );
                }
            }
        );

    updatePipelineLinks();
}


function resetPipelineMonitor() {
    dashboardState.lastPipelineSequence =
        0;

    document
        .querySelectorAll(
            ".pipeline-stage"
        )
        .forEach(
            stage => {
                stage.classList.remove(
                    "waiting",
                    "running",
                    "completed",
                    "failed",
                    "skipped",
                    "artifact-pulse",
                );

                stage.classList.add(
                    "waiting"
                );

                const status =
                    stage.querySelector(
                        ".stage-text span"
                    );

                if (status) {
                    status.textContent =
                        "Waiting";
                }
            }
        );

    document
        .querySelectorAll(
            ".pipeline-link"
        )
        .forEach(
            link => {
                link.classList.remove(
                    "active",
                    "completed",
                    "skipped",
                );
            }
        );

    elements.pipelineTelemetryStatus.textContent =
        "WAITING FOR TELEMETRY";

    elements.pipelineTelemetryStatus.classList.remove(
        "live"
    );

    applyPipelineMode(
        dashboardState.preview
            ?.perception_mode
        ?? "generalized"
    );
}


function updatePipelineLinks() {
    const generalized =
        (
            dashboardState.preview
                ?.perception_mode
            ?? "generalized"
        )
        === "generalized";

    document
        .querySelectorAll(
            ".pipeline-link"
        )
        .forEach(
            link => {
                const generalizedOnly =
                    (
                        link.dataset
                            .generalizedOnly
                        === "true"
                    );

                link.classList.remove(
                    "active",
                    "completed",
                );

                if (
                    generalizedOnly
                    && !generalized
                ) {
                    link.classList.add(
                        "skipped"
                    );

                    return;
                }

                link.classList.remove(
                    "skipped"
                );

                const from =
                    getPipelineStage(
                        link.dataset.from
                    );

                const to =
                    getPipelineStage(
                        link.dataset.to
                    );

                if (
                    !from
                    || !to
                ) {
                    return;
                }

                if (
                    from.classList.contains(
                        "completed"
                    )
                ) {
                    link.classList.add(
                        "completed"
                    );
                }

                if (
                    to.classList.contains(
                        "running"
                    )
                ) {
                    link.classList.add(
                        "active"
                    );
                }
            }
        );
}


function setPipelineStageState(stageName, state) {
    const stage =
        getPipelineStage(
            stageName
        );

    if (!stage) {
        console.warn(
            "Unknown pipeline stage:",
            stageName
        );

        return;
    }

    stage.classList.remove(
        "skipped"
    );

    stage.classList.remove(
        "waiting",
        "running",
        "completed",
        "failed",
    );

    stage.classList.add(
        state
    );

    const status =
        stage.querySelector(
            ".stage-text span"
        );

    if (status) {
        switch (state) {

            case "running":
                status.textContent =
                    "Processing";
                break;

            case "completed":
                status.textContent =
                    "Completed";
                break;

            case "failed":
                status.textContent =
                    "Failed";
                break;

            default:
                status.textContent =
                    "Waiting";
        }
    }

    updatePipelineLinks();
}


function pulsePipelineArtifact(stageName) {
    const stage =
        getPipelineStage(
            stageName
        );

    if (!stage) {
        return;
    }

    stage.classList.remove(
        "artifact-pulse"
    );

    void stage.offsetWidth;

    stage.classList.add(
        "artifact-pulse"
    );

    setTimeout(
        () => {
            stage.classList.remove(
                "artifact-pulse"
            );
        },
        650,
    );
}


function handlePipelineEvent(event) {
    dashboardState.lastPipelineSequence =
        Math.max(
            dashboardState.lastPipelineSequence,
            event.sequence ?? 0,
        );

    elements.pipelineTelemetryStatus.textContent =
        "TELEMETRY LIVE";

    elements.pipelineTelemetryStatus.classList.add(
        "live"
    );

    if (
        event.event === "stage_started"
        && event.stage
    ) {
        setPipelineStageState(
            event.stage,
            "running",
        );

        enqueueVisualPipelineEvent(
            event
        );

        return;
    }

    if (
        event.event === "stage_completed"
        && event.stage
    ) {
        setPipelineStageState(
            event.stage,
            "completed",
        );

        enqueueVisualPipelineEvent(
            event
        );

        return;
    }

    if (
        (
            event.event === "stage_failed"
            || event.event === "pipeline_error"
        )
        && event.stage
    ) {
        setPipelineStageState(
            event.stage,
            "failed",
        );

        enqueueVisualPipelineEvent(
            event
        );

        return;
    }

    if (
        event.event === "artifact_created"
        && event.stage
    ) {
        pulsePipelineArtifact(
            event.stage
        );

        if (
            event.artifact_type
            === "action_plan"
        ) {
            renderActionPlanArtifact(
                event
            );
        }
    }
}


async function pollPipelineEvents() {
    try {
        const response =
            await fetch(
                `/api/pipeline/events?after_sequence=${dashboardState.lastPipelineSequence}`
            );

        if (!response.ok) {
            return;
        }

        const payload =
            await response.json();

        for (
            const event
            of payload.events ?? []
        ) {
            handlePipelineEvent(
                event
            );
        }
    }

    catch (error) {
        console.error(
            "Pipeline event polling failed:",
            error
        );
    }
}


/* ============================================================
   RUNTIME VISUAL / TERMINAL
   ============================================================ */


const VISUAL_STAGE_DEFINITIONS = {
    keyframe_selector: {
        index: "01",

        kicker:
            "DEMONSTRATION UNDERSTANDING",

        title:
            "Keyframe Selector",

        running:
            "Selecting the most informative frames from the demonstration...",

        completed:
            "Keyframe selection completed.",
    },

    visual_prompter: {
        index: "02",

        kicker:
            "DEMONSTRATION UNDERSTANDING",

        title:
            "Visual Prompter",

        running:
            "Tracking and grounding the relevant objects in the demonstration...",

        completed:
            "Visual prompting completed.",
    },

    demo_structured_scene_builder: {
        index: "03",

        kicker:
            "DEMONSTRATION UNDERSTANDING",

        title:
            "Demo Structured Scene",

        running:
            "Building the structured representation of the demonstrated scene...",

        completed:
            "Demonstration scene representation completed.",
    },

    action_planner: {
        index: "04",

        kicker:
            "DEMONSTRATION UNDERSTANDING",

        title:
            "Action Planner",

        running:
            "Inferring the task and generating the action plan...",

        completed:
            "Action planning completed.",
    },

    scene_perceiver: {
        index: "05",

        kicker:
            "RUNTIME UNDERSTANDING",

        title:
            "Scene Perceiver",

        running:
            "Perceiving the current robot workspace...",

        completed:
            "Runtime scene perception completed.",
    },

    scene_interpreter: {
        index: "06",

        kicker:
            "RUNTIME UNDERSTANDING",

        title:
            "Scene Interpreter",

        running:
            "Interpreting objects, attributes and spatial relationships...",

        completed:
            "Runtime scene interpretation completed.",
    },

    runtime_structured_scene_builder: {
        index: "07",

        kicker:
            "RUNTIME UNDERSTANDING",

        title:
            "Runtime Structured Scene",

        running:
            "Building the structured representation of the runtime scene...",

        completed:
            "Runtime scene representation completed.",
    },

    structural_matcher: {
        index: "08",

        kicker:
            "TASK TRANSFER",

        title:
            "Structural Matcher",

        running:
            "Matching the demonstrated scene structure to the runtime scene...",

        completed:
            "Structural matching completed.",
    },

    replicability_checker: {
        index: "09",

        kicker:
            "TASK TRANSFER",

        title:
            "Replicability Checker",

        running:
            "Checking whether the demonstrated task can be reproduced...",

        completed:
            "Task replicability confirmed.",
    },

    lmp_generator: {
        index: "10",

        kicker:
            "MOTION GENERATION",

        title:
            "LMP Generator",

        running:
            "Generating the executable primitive plan...",

        completed:
            "Primitive plan generated.",
    },

    robot_execution: {
        index: "11",

        kicker:
            "PHYSICAL EXECUTION",

        title:
            "Robot Execution",

        running:
            "Executing the generated plan on the robot...",

        completed:
            "Robot execution completed.",
    },
};


const VISUAL_COMPLETION_HOLD_MS =
    650;


function setRuntimeView(mode) {
    dashboardState.runtimeView =
        mode;

    const visual =
        mode === "visual";

    elements.runtimeVisualView.classList.toggle(
        "hidden",
        !visual,
    );

    elements.runtimeTerminalView.classList.toggle(
        "hidden",
        visual,
    );

    elements.runtimeVisualButton.classList.toggle(
        "active",
        visual,
    );

    elements.runtimeTerminalButton.classList.toggle(
        "active",
        !visual,
    );

    elements.runtimeViewCaption.textContent =
        visual
            ? "Live experiment visualization"
            : "Raw SeeDo runtime output";
}


function delay(milliseconds) {
    return new Promise(
        resolve => {
            setTimeout(
                resolve,
                milliseconds,
            );
        }
    );
}


function getVisualStageDefinition(stageName) {
    return (
        VISUAL_STAGE_DEFINITIONS[
            stageName
        ]
        ?? {
            index: "—",

            kicker:
                "SEEDO PIPELINE",

            title:
                stageName
                    ?.replaceAll(
                        "_",
                        " "
                    )
                ?? "Pipeline stage",

            running:
                "Processing pipeline stage...",

            completed:
                "Pipeline stage completed.",
        }
    );
}


function stopRobotCameraStream() {
    dashboardState.cameraStreamActive =
        false;

    elements.robotCameraStream.removeAttribute(
        "src"
    );

    elements.robotCameraState.textContent =
        "OFFLINE";

    elements.robotCameraState.classList.remove(
        "live",
        "failed",
    );
}


function startRobotCameraStream() {
    if (
        dashboardState.cameraStreamActive
    ) {
        return;
    }

    dashboardState.cameraStreamActive =
        true;

    elements.robotCameraState.textContent =
        "CONNECTING";

    elements.robotCameraState.classList.remove(
        "live",
        "failed",
    );

    elements.robotCameraStream.src =
        (
            "/api/camera/front/stream"
            + `?run=${Date.now()}`
        );
}


function setCurrentStageState(
    stageName,
    state,
    message = null,
) {
    const definition =
        getVisualStageDefinition(
            stageName
        );

    dashboardState.activeVisualStage =
        stageName;

    elements.currentStageIndex.textContent =
        definition.index;

    elements.currentStageKicker.textContent =
        definition.kicker;

    elements.currentStageTitle.textContent =
        definition.title;

    elements.currentStageCard.classList.remove(
        "waiting",
        "running",
        "completed",
        "failed",
    );

    elements.currentStageCard.classList.add(
        state
    );

    if (
        state === "running"
    ) {
        elements.currentStageBadge.textContent =
            "RUNNING";

        elements.currentStageStatus.textContent =
            message
            ?? definition.running;

        elements.currentStageCore.textContent =
            definition.index;
    }

    else if (
        state === "completed"
    ) {
        elements.currentStageBadge.textContent =
            "COMPLETED";

        elements.currentStageStatus.textContent =
            message
            ?? definition.completed;

        elements.currentStageCore.textContent =
            "✓";
    }

    else if (
        state === "failed"
    ) {
        elements.currentStageBadge.textContent =
            "FAILED";

        elements.currentStageStatus.textContent =
            message
            ?? "The current pipeline stage failed.";

        elements.currentStageCore.textContent =
            "!";
    }

    else {
        elements.currentStageBadge.textContent =
            "WAITING";

        elements.currentStageStatus.textContent =
            message
            ?? "Waiting for pipeline telemetry...";

        elements.currentStageCore.textContent =
            definition.index;
    }

    const robotExecution =
        stageName
        === "robot_execution";

    elements.currentStageAnimation.classList.toggle(
        "hidden",
        robotExecution,
    );

    elements.robotCameraPanel.classList.toggle(
        "hidden",
        !robotExecution,
    );

    if (robotExecution) {
        startRobotCameraStream();
    }

    else {
        stopRobotCameraStream();
    }
}


function resetRuntimeVisual() {
    dashboardState.visualEventQueue =
        [];

    dashboardState.visualEventProcessing =
        false;

    dashboardState.visualQueueGeneration +=
        1;

    dashboardState.activeVisualStage =
        null;

    elements.visualWorkspace.classList.remove(
        "hidden"
    );

    elements.visualErrorCard.classList.add(
        "hidden"
    );

    elements.visualErrorCard.classList.remove(
        "active",
        "failed",
    );

    elements.visualErrorMessage.textContent =
        "";

    elements.visualErrorReturnCode.textContent =
        "—";

    elements.demoVideoState.textContent =
        "LOADING";

    elements.demoVideoState.className =
        "artifact-state loading";

    elements.actionPlanCard.classList.remove(
        "ready",
        "failed",
    );

    elements.actionPlanCard.classList.add(
        "waiting"
    );

    elements.actionPlanState.textContent =
        "WAITING";

    elements.actionPlanState.className =
        "artifact-state";

    elements.actionPlanText.textContent =
        "Waiting for the Action Planner...";

    elements.currentStageCard.classList.remove(
        "waiting",
        "completed",
        "failed",
    );

    elements.currentStageCard.classList.add(
        "running"
    );

    elements.currentStageIndex.textContent =
        "SETUP";

    elements.currentStageKicker.textContent =
        "EXPERIMENT INITIALIZATION";

    elements.currentStageTitle.textContent =
        "Preparing experiment";

    elements.currentStageStatus.textContent =
        "Initializing robot interfaces and runtime services...";

    elements.currentStageBadge.textContent =
        "RUNNING";

    elements.currentStageCore.textContent =
        "·";

    elements.currentStageAnimation.classList.remove(
        "hidden"
    );

    elements.robotCameraPanel.classList.add(
        "hidden"
    );

    stopRobotCameraStream();

    elements.demoVideo.pause();

    elements.demoVideo.removeAttribute(
        "src"
    );

    elements.demoVideo.load();
}


function prepareDemoVideo() {
    if (
        !dashboardState.activeExperiment
    ) {
        return;
    }

    elements.demoVideoState.textContent =
        "LOADING";

    elements.demoVideoState.className =
        "artifact-state loading";

    elements.demoVideo.src =
        `/api/experiment/demo-video?run=${Date.now()}`;

    elements.demoVideo.load();
}


function renderActionPlanArtifact(event) {
    const content =
        (
            event.content
            ?? event.natural_language_plan
            ?? ""
        )
            .toString()
            .trim();

    if (!content) {
        return;
    }

    elements.actionPlanText.textContent =
        content;

    elements.actionPlanState.textContent =
        "READY";

    elements.actionPlanState.className =
        "artifact-state ready";

    elements.actionPlanCard.classList.remove(
        "waiting",
        "failed",
    );

    elements.actionPlanCard.classList.add(
        "ready"
    );
}


function enqueueVisualPipelineEvent(event) {
    dashboardState.visualEventQueue.push(
        event
    );

    processVisualEventQueue();
}


async function processVisualEventQueue() {
    if (
        dashboardState.visualEventProcessing
    ) {
        return;
    }

    dashboardState.visualEventProcessing =
        true;

    const generation =
        dashboardState.visualQueueGeneration;

    try {
        while (
            dashboardState.visualEventQueue.length
            > 0
        ) {
            if (
                generation
                !== dashboardState.visualQueueGeneration
            ) {
                break;
            }

            const event =
                dashboardState
                    .visualEventQueue
                    .shift();

            if (
                !event
                || !event.stage
            ) {
                continue;
            }

            if (
                event.event
                === "stage_started"
            ) {
                setCurrentStageState(
                    event.stage,
                    "running",
                );

                setRuntimeView(
                    "visual"
                );

                continue;
            }

            if (
                event.event
                === "stage_completed"
            ) {
                setCurrentStageState(
                    event.stage,
                    "completed",
                );

                await delay(
                    VISUAL_COMPLETION_HOLD_MS
                );

                continue;
            }

            if (
                event.event
                === "stage_failed"
                ||
                event.event
                === "pipeline_error"
            ) {
                setCurrentStageState(
                    event.stage,
                    "failed",
                    event.message
                    ?? null,
                );

                await delay(
                    VISUAL_COMPLETION_HOLD_MS
                );
            }
        }
    }

    finally {
        dashboardState.visualEventProcessing =
            false;

        if (
            dashboardState.visualEventQueue.length
            > 0
        ) {
            processVisualEventQueue();
        }
    }
}


function hideExecutionError() {
    elements.executionErrorOverlay.classList.remove(
        "visible"
    );
}


function showVisualError(process) {
    const message =
        process.last_error
        ??
        (
            "Experiment process exited "
            + `with code ${process.return_code ?? "unknown"}.`
        );

    dashboardState.visualEventQueue =
        [];

    dashboardState.visualQueueGeneration +=
        1;

    stopRobotCameraStream();

    elements.visualWorkspace.classList.add(
        "hidden"
    );

    elements.visualErrorMessage.textContent =
        message;

    elements.visualErrorReturnCode.textContent =
        process.return_code
        ?? "—";

    elements.visualErrorCard.classList.remove(
        "hidden"
    );

    elements.visualErrorCard.classList.add(
        "failed"
    );

    setRuntimeView(
        "visual"
    );
}


function showExecutionError(process) {
    const message =
        process.last_error
        ??
        (
            "Experiment process exited "
            + `with code ${process.return_code ?? "unknown"}.`
        );

    elements.executionErrorTitle.textContent =
        "Experiment failed";

    elements.executionErrorMessage.textContent =
        message;

    elements.executionErrorReturnCode.textContent =
        process.return_code
        ?? "—";

    elements.executionErrorOverlay.classList.add(
        "visible"
    );
}


/* ============================================================
   PROCESS / RUNTIME
   ============================================================ */


function updateProcessStatus(process) {
    const status =
        process.status
        ?? "idle";

    dashboardState.processStatus =
        status;

    elements.processStatus.textContent =
        status.toUpperCase();

    elements.processStatus.className =
        `process-badge ${status}`;

    const showRuntime =
        status !== "idle";

    elements.runtimeIdle.classList.toggle(
        "hidden",
        showRuntime,
    );

    elements.runtimeActive.classList.toggle(
        "hidden",
        !showRuntime,
    );

    elements.runtimePid.textContent =
        process.pid ?? "—";

    const running =
        (
            status === "running"
            || status === "starting"
        );

    elements.runtimeLiveIndicator.classList.toggle(
        "visible",
        running,
    );

    elements.terminalStreamIndicator.classList.toggle(
        "visible",
        running,
    );

    if (
        status === "failed"
    ) {
        const errorSignature =
            [
                process.return_code,
                process.last_error,
            ].join("|");

        if (
            dashboardState.lastErrorSignature
            !== errorSignature
        ) {
            dashboardState.lastErrorSignature =
                errorSignature;

            showVisualError(
                process
            );

            showExecutionError(
                process
            );
        }
    }

    updateStartAvailability();
}


function appendLogs(logs) {
    if (
        !logs.length
    ) {
        return;
    }

    const terminalBody =
        elements.runtimeConsole.parentElement;

    const distanceFromBottom =
        terminalBody.scrollHeight
        - terminalBody.scrollTop
        - terminalBody.clientHeight;

    const shouldAutoScroll =
        distanceFromBottom
        < 60;

    for (
        const entry
        of logs
    ) {
        elements.runtimeConsole.textContent +=
            entry.line + "\n";

        dashboardState.lastLogSequence =
            Math.max(
                dashboardState.lastLogSequence,
                entry.sequence,
            );
    }

    if (
        shouldAutoScroll
    ) {
        terminalBody.scrollTop =
            terminalBody.scrollHeight;
    }
}


/* ============================================================
   USER INTERACTION
   ============================================================ */


function showInteraction(interaction) {
    dashboardState.currentInteractionId =
        interaction.interaction_id;

    dashboardState.currentInteractionType =
        interaction.interaction_type
        ?? "text";

    const enterOnly =
        dashboardState.currentInteractionType
        === "enter";

    elements.interactionPrompt.textContent =
        interaction.prompt;

    elements.interactionValue.value =
        "";

    elements.interactionResponseField.classList.toggle(
        "hidden",
        enterOnly,
    );

    elements.interactionSubmit.textContent =
        enterOnly
            ? "Press Enter"
            : "Send Response";

    elements.interactionError.classList.remove(
        "visible"
    );

    elements.interactionOverlay.classList.add(
        "visible"
    );

    setTimeout(
        () => {
            if (enterOnly) {
                elements.interactionSubmit.focus();
            }

            else {
                elements.interactionValue.focus();
            }
        },
        260,
    );
}


function hideInteraction() {
    dashboardState.currentInteractionId =
        null;

    dashboardState.currentInteractionType =
        "text";

    elements.interactionResponseField.classList.remove(
        "hidden"
    );

    elements.interactionSubmit.textContent =
        "Send Response";

    elements.interactionOverlay.classList.remove(
        "visible"
    );
}


async function pollInteraction() {
    try {
        const response =
            await fetch(
                "/api/interactions/pending"
            );

        if (
            !response.ok
        ) {
            return;
        }

        const payload =
            await response.json();

        const interaction =
            payload.interaction;

        if (!interaction) {
            if (
                dashboardState.currentInteractionId
                !== null
            ) {
                hideInteraction();
            }

            return;
        }

        if (
            interaction.interaction_id
            !== dashboardState.currentInteractionId
        ) {
            showInteraction(
                interaction
            );
        }
    }

    catch (error) {
        console.error(
            "Interaction polling failed:",
            error
        );
    }
}


async function submitInteraction() {
    const interactionId =
        dashboardState.currentInteractionId;

    if (
        !interactionId
    ) {
        return;
    }

    elements.interactionSubmit.disabled =
        true;

    const value =
        dashboardState.currentInteractionType
        === "enter"
            ? ""
            : elements.interactionValue.value;

    try {
        const response =
            await fetch(
                `/api/interactions/${interactionId}/respond`,
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "application/json",
                    },

                    body:
                        JSON.stringify({
                            value: value,
                        }),
                }
            );

        const payload =
            await response.json();

        if (
            !response.ok
            || payload.status !== "ok"
        ) {
            throw new Error(
                payload.detail
                ?? payload.message
                ?? "Failed to send response."
            );
        }

        hideInteraction();
    }

    catch (error) {
        elements.interactionError.textContent =
            error.message;

        elements.interactionError.classList.add(
            "visible"
        );
    }

    finally {
        elements.interactionSubmit.disabled =
            false;
    }
}


/* ============================================================
   POLLING
   ============================================================ */


async function pollRuntime() {
    try {
        const [
            statusResponse,
            logsResponse,
        ] =
            await Promise.all([
                fetch(
                    "/api/process/status"
                ),

                fetch(
                    `/api/process/logs?after_sequence=${dashboardState.lastLogSequence}`
                ),
            ]);

        if (
            statusResponse.ok
        ) {
            const process =
                await statusResponse.json();

            updateProcessStatus(
                process
            );
        }

        if (
            logsResponse.ok
        ) {
            const payload =
                await logsResponse.json();

            appendLogs(
                payload.logs ?? []
            );
        }

        await pollInteraction();

        await pollPipelineEvents();
    }

    catch (error) {
        console.error(
            "Runtime polling failed:",
            error
        );
    }
}


/* ============================================================
   START / STOP
   ============================================================ */


async function startExperiment() {
    if (
        !dashboardState.preview
        || processIsBusy()
    ) {
        return;
    }

    elements.startButton.disabled =
        true;

    elements.startButtonLabel.textContent =
        "Starting...";

    dashboardState.lastLogSequence =
        0;

    resetPipelineMonitor();

    elements.runtimeConsole.textContent =
        "";

    dashboardState.activeExperiment =
        null;

    dashboardState.lastErrorSignature =
        null;

    hideExecutionError();

    setRuntimeView(
        "visual"
    );

    resetRuntimeVisual();

    try {
        const response =
            await fetch(
                "/api/experiment/start",
                {
                    method: "POST",

                    headers: {
                        "Content-Type":
                            "application/json",
                    },

                    body:
                        JSON.stringify(
                            buildRequest()
                        ),
                }
            );

        const payload =
            await response.json();

        if (
            !response.ok
        ) {
            throw new Error(
                payload.detail
                ?? "Failed to start experiment."
            );
        }

        dashboardState.activeExperiment =
            payload.experiment
            ?? null;

        prepareDemoVideo();

        updateProcessStatus(
            payload.process
        );

        await pollRuntime();
    }

    catch (error) {
        setPreviewError(
            error.message
        );
    }

    finally {
        elements.startButtonLabel.textContent =
            "Start Experiment";

        updateStartAvailability();
    }
}


async function stopExperiment() {
    if (
        !processIsBusy()
    ) {
        return;
    }

    elements.stopButton.disabled =
        true;

    try {
        const response =
            await fetch(
                "/api/process/stop",
                {
                    method: "POST",
                }
            );

        if (
            !response.ok
        ) {
            throw new Error(
                "Failed to stop experiment."
            );
        }

        await pollRuntime();
    }

    catch (error) {
        console.error(
            "Stop failed:",
            error
        );
    }

    finally {
        elements.stopButton.disabled =
            false;
    }
}


/* ============================================================
   EVENT LISTENERS
   ============================================================ */


function attachInteractions() {
    const formElements = [
        elements.controllerSelect,
        elements.taskSelect,
        elements.executionTypeSelect,
        elements.demoTask,
        elements.demoPath,
        elements.configureScene,
        elements.sceneTask,
        elements.sceneTrajectory,
        elements.precomputedPlan,
        elements.precomputedDemoScene,
    ];

    for (
        const element
        of formElements
    ) {
        element.addEventListener(
            "input",
            schedulePreview,
        );

        element.addEventListener(
            "change",
            schedulePreview,
        );
    }

    elements.taskSelect.addEventListener(
        "change",
        () => {
            updateTaskDependentFields();

            schedulePreview();
        },
    );

    elements.executionTypeSelect.addEventListener(
        "change",
        () => {
            updateExecutionTypeVisibility();

            schedulePreview();
        },
    );

    elements.configureScene.addEventListener(
        "change",
        () => {
            updateIsaacVisibility();

            schedulePreview();
        },
    );

    elements.startButton.addEventListener(
        "click",
        startExperiment,
    );

    elements.stopButton.addEventListener(
        "click",
        stopExperiment,
    );

    elements.interactionSubmit.addEventListener(
        "click",
        submitInteraction,
    );

    elements.interactionValue.addEventListener(
        "keydown",
        event => {
            if (
                event.key === "Enter"
            ) {
                event.preventDefault();

                submitInteraction();
            }
        },
    );

    elements.runtimeVisualButton.addEventListener(
        "click",
        () => {
            setRuntimeView(
                "visual"
            );
        },
    );

    elements.runtimeTerminalButton.addEventListener(
        "click",
        () => {
            setRuntimeView(
                "terminal"
            );
        },
    );

    elements.executionErrorClose.addEventListener(
        "click",
        hideExecutionError,
    );

    elements.visualErrorTerminal.addEventListener(
        "click",
        () => {
            setRuntimeView(
                "terminal"
            );
        },
    );

    elements.executionErrorTerminal.addEventListener(
        "click",
        () => {
            hideExecutionError();

            setRuntimeView(
                "terminal"
            );
        },
    );

    elements.demoVideo.addEventListener(
        "loadeddata",
        () => {
            elements.demoVideoState.textContent =
                "READY";

            elements.demoVideoState.className =
                "artifact-state ready";

            elements.demoVideo
                .play()
                .catch(
                    () => {
                        // Browser autoplay may be blocked.
                    }
                );
        },
    );

    elements.demoVideo.addEventListener(
        "error",
        () => {
            elements.demoVideoState.textContent =
                "UNAVAILABLE";

            elements.demoVideoState.className =
                "artifact-state failed";
        },
    );

    elements.robotCameraStream.addEventListener(
        "load",
        () => {
            elements.robotCameraState.textContent =
                "LIVE";

            elements.robotCameraState.classList.add(
                "live"
            );

            elements.robotCameraState.classList.remove(
                "failed"
            );
        },
    );

    elements.robotCameraStream.addEventListener(
        "error",
        () => {
            if (
                dashboardState.cameraStreamActive
            ) {
                elements.robotCameraState.textContent =
                    "UNAVAILABLE";

                elements.robotCameraState.classList.remove(
                    "live"
                );

                elements.robotCameraState.classList.add(
                    "failed"
                );
            }
        },
    );
}


/* ============================================================
   INITIALIZATION
   ============================================================ */


async function loadDashboard() {
    try {
        const [
            healthResponse,
            optionsResponse,
            processResponse,
        ] =
            await Promise.all([
                fetch(
                    "/api/health"
                ),

                fetch(
                    "/api/experiment/options"
                ),

                fetch(
                    "/api/process/status"
                ),
            ]);

        if (
            !healthResponse.ok
            || !optionsResponse.ok
            || !processResponse.ok
        ) {
            throw new Error(
                "Backend returned an error."
            );
        }

        const health =
            await healthResponse.json();

        const options =
            await optionsResponse.json();

        const process =
            await processResponse.json();

        dashboardState.options =
            options;

        setConnectionState(
            true,
            health.ros_available
                ? "Backend online · ROS 2 available"
                : "Backend online · ROS 2 unavailable",
        );

        populateSelect(
            elements.controllerSelect,
            options.controllers,
            options.defaults.controller,
        );

        populateSelect(
            elements.taskSelect,
            options.tasks,
            options.defaults.task,
        );

        populateSelect(
            elements.executionTypeSelect,
            options.execution_types,
            options.defaults.execution_type,
        );

        elements.executionTarget.textContent =
            formatExecutionMode(
                options.environment
                    .execution_mode
            );

        elements.configPath.textContent =
            options.defaults.config;

        elements.demoTask.value =
            options.defaults.demo_task;

        elements.demoPath.value =
            options.defaults.demo_path
            ?? "";

        elements.configureScene.checked =
            options.defaults
                .configure_isaac_scene;

        elements.sceneTask.value =
            options.defaults.scene_task
            ?? 0;

        elements.sceneTrajectory.value =
            options.defaults
                .scene_trajectory
            ?? 0;

        elements.precomputedPlan.value =
            options.defaults
                .precomputed_plan
            ?? "";

        elements.precomputedDemoScene.value =
            options.defaults
                .precomputed_demo_scene
            ?? "";

        updateTaskDependentFields();

        updateIsaacVisibility();

        updateExecutionTypeVisibility();

        updateProcessStatus(
            process
        );

        resetPipelineMonitor();

        resetRuntimeVisual();

        attachInteractions();

        await refreshPreview();

        await pollRuntime();

        dashboardState.pollTimer =
            setInterval(
                pollRuntime,
                650,
            );
    }

    catch (error) {
        console.error(
            error
        );

        setConnectionState(
            false,
            "Backend unavailable",
        );
    }
}


loadDashboard();