# UR5e-2f-85 🤖

**A real robot, six brains.** Dockerized ROS 2 (Jazzy) stack for a UR5e arm
fitted with a Robotiq 2F-85 gripper and a 4-camera ZED rig — teleoperate it,
collect demonstrations, or hand control to one of six interchangeable
imitation-learning policies (from a classic conditioned-detector to a
VLM-driven VLA-JEPA policy captioned on the fly by NVIDIA Cosmos) and watch
it pick, place, and sort boxes on its own.

Everything here runs on physical hardware — no simulator required — and is
built to run equally well on a workstation GPU or an NVIDIA DGX Spark.

## Highlights

- 🐳 **One repo, one set of Dockerfiles** for the whole stack: ROS 2 base,
  UR-driver + gripper + table, teleoperation, dataset collection, and ZED
  camera drivers.
- 🕹️ **Teleoperate** the real arm+gripper today, **collect trajectories**
  tomorrow, **replay** them whenever you need a regression check.
- 🧠 **Six AI controllers, one interface** — swap policies with a single
  ROS parameter and let `ai_controller_node` handle cameras, robot state,
  and safety-bounded actions for every one of them.
- 🎯 **A non-learned fallback**: `script_controller` does click-to-target
  pick-place with hand-coded primitives, for when you just need the robot
  to *do the thing*.
- 📷 Multi-camera ZED rig (front / left / right / wrist) with calibration
  tooling and depth-aware grasp refinement baked into the AI controllers
  that need it.

## The AI Controllers

Every controller below plugs into the same `ai_controller_node` launch
pattern — front/side/wrist camera images and robot state in, a
safety-clamped end-effector pose + gripper command out.

| Controller | What it is | Docs |
|---|---|---|
| **COD** | Conditioned-target-object-detector MOSAIC double-policy, with an optional eye-in-hand stream | [cod_controller.md](docs/ai_controller_models/cod_controller.md) |
| **OpenVLA** | [openvla-oft](https://github.com/ciccio42/openvla-oft) fine-tuned for closed-loop pick-place | [openvla_controller.md](docs/ai_controller_models/openvla_controller.md) |
| **TinyVLA** | LoRA-fine-tuned Llava-Pythia VLM with an ACT/diffusion action head | [tinyvla_controller.md](docs/ai_controller_models/tinyvla_controller.md) |
| **OSVI-WM** | World-model-based policy | [osvi_controller.md](docs/ai_controller_models/osvi_controller.md) |
| **OSVI-AWDA** | Predicts 5 image-space waypoints per glance, then refines the grasp with wrist depth | [osvi_awda_controller.md](docs/ai_controller_models/osvi_awda_controller.md) |
| **VLA-JEPA + Cosmos** | A `lerobot`/V-JEPA2 policy whose task instruction can be generated on the fly by NVIDIA **Cosmos-Reason2**, captioning a human demo instead of reading a canned prompt | [video_captioning.md](docs/ai_controller_models/video_captioning.md) |

*(VLA-JEPA runs behind a small HTTP bridge — its dependencies need NumPy 2,
which conflicts with ROS's `cv_bridge`, so it lives in its own venv and
talks to `ai_controller_node` over loopback. Details in the doc above.)*

## Docs

**Setup**
- [ROS2](docs/ros2.md) — base ROS 2 Jazzy image
- [UR](docs/ur.md) — arm-only docker
- [UR-Robotiq](docs/ur_robotiq.md) — arm + gripper + table
- [Bringup](docs/bringup.md) — sim and real robot bringup
- [ZED](docs/zed.md) — camera drivers and calibration

**Operate**
- [Teleoperation](docs/teleoperation.md) — drive the real arm+gripper
- [Script-Controller](docs/script_controller.md) — scripted, click-to-target pick-place, no learned model
- [AI-Controller](docs/ai_controller.md) — shared launch setup for every learned policy above

**Data**
- [Dataset-Collector](docs/dataset_collector.md) — record trajectories for training

**Reference**
- [Useful commands](docs/useful_commands.md)
- [Utils](docs/utils.md)
- [ToDo](docs/todo.md)

## Repo layout

```
UR5e-2f-85/
├── ur5e_2f_85/         # robot description, MoveIt config, teleoperation
├── moveit_controller/  # ROS2 node: GoHome/GoToPose services on top of MoveIt
├── ai_controller/       # ai_controller_node + every policy in the table above
├── dataset_collector/   # trajectory recording/replay
├── zed_camera/          # ZED drivers, calibration, multi-camera launch
├── docker/              # every Dockerfile + entrypoint used by docs/*.md
└── docs/                 # one page per component (linked above)
```
