# Starting the SeeDo + Isaac Sim Simulation

## 1. Isaac Sim

On the host system:

```bash
cd /home/asus-mivia/Desktop/Isaac-Sim/Nvidia-Isaac-Sim-Env
```

Start the Isaac container:

```bash
xhost +local:

docker run --name isaac-sim-ur5e \
  -it \
  --rm \
  --net=host \
  --ipc=host \
  --gpus all \
  -e ACCEPT_EULA=Y \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PRIVACY_CONSENT=Y \
  -v "$HOME/.Xauthority:/isaac-sim/.Xauthority" \
  -e DISPLAY \
  -v "$HOME/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw" \
  -v "$HOME/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$HOME/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$HOME/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$HOME/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$HOME/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$PWD/usd:/isaac-sim/usd" \
  -v "$PWD/docker:/workspace/isaac_tools:ro" \
  isaac-sim-ur5e:5.1.0
```

Inside the container:

```bash
cd /isaac-sim/usd/ur5e_2f_85

source /opt/ros/jazzy/setup.bash
source /workspace/jazzy_ws/install/setup.bash
source /workspace/build_ws/install/setup.bash

export ROS_PACKAGE_PATH="/workspace/build_ws/install/share:/workspace/jazzy_ws/install/share:/opt/ros/jazzy/share"

/isaac-sim/isaac-sim.sh --allow-root \
  --ext-folder /workspace/isaac_tools/exts \
  --enable seedo.scene_config \
  --enable isaacsim.asset.importer.urdf \
  --enable isaacsim.asset.exporter.urdf \
  --enable isaacsim.ros2.urdf \
  --enable isaacsim.ros2.bridge
```

Open:

```text
/isaac-sim/usd/ur5e_2f_85/seedo_scene_v3.usd
```

and press **Play**.

---

## 2. SeeDo

On the host system:

```bash
cd /home/asus-mivia/Desktop/Angelo/UR5e-2f-85
```

Start the SeeDo container:

```bash
docker run --rm -it \
  --name seedo_ros2_container \
  --gpus all \
  --network host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/home/ros2_ws/src/UR5e-2f-85 \
  -v "$(pwd)/.runtime/graspmolmo_venv":/opt/graspmolmo_venv \
  -v "$(pwd)/external/GraspMolmo":/opt/GraspMolmo \
  -v "$(pwd)/.runtime/m2t2_venv":/opt/m2t2_venv \
  -v "$(pwd)/external/M2T2":/opt/M2T2 \
  -v "$(pwd)/external/GraspGenX":/opt/GraspGenX \
  -v "$(pwd)/.runtime/graspgenx_venv":/opt/graspgenx_venv \
  -v "$(pwd)/.runtime/graspgenx_checkpoints":/opt/graspgenx_checkpoints \
  -v "$(pwd)/.runtime/graspgenx_gripper_descriptions":/opt/graspgenx_gripper_descriptions \
  -v "$(pwd)/../dataset_nut_assembly/video":/dataset_nut:ro \
  -v "$(pwd)/../test_dataset":/test_dataset:ro \
  -v "$(pwd)/../seedo_tests":/seedo_tests \
  -v "$(pwd)/../scene_capture":/scene_capture:ro \
  -v "$(pwd)/../test_isaac":/test_isaac \
  -v "$(pwd)/../test_real":/test_real \
  -v "$(pwd)/../full_nut_case":/full_nut_case \
  -v "$(pwd)/saved_trajectories":/home/saved_trajectories \
  -v "$(pwd)/../Experiments":/Experiments \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENAI_API_KEY="$OPENAI_API_KEY" \
  -e BUILD_SEEDO_ROS=1 \
  -e ROS_DOMAIN_ID=42 \
  seedo_ros2:latest
```

## Terminal 1 — Start the GraspMolmo server

```bash
docker exec -it seedo_ros2_container bash

cd /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/utils/grasping/graspmolmo

HF_HOME=/home/ros2_ws/src/UR5e-2f-85/.runtime/huggingface \
PYTHONPATH=/home/ros2_ws/src/UR5e-2f-85/ai_controller \
PYTHONNOUSERSITE=1 \
/opt/graspmolmo_venv/bin/python server.py \
    --config config/graspmolmo_config.yaml
```

---

## Terminal 2 — Start the GraspGen-X server

```bash
docker exec -it seedo_ros2_container bash

cd /opt/GraspGenX

env -u PYTHONPATH \
PYTHONNOUSERSITE=1 \
GRASPGENX_CHECKPOINT_DIR=/opt/graspgenx_checkpoints \
GRASPGENX_GRIPPER_CFG_DIR=/opt/graspgenx_gripper_descriptions \
/opt/graspgenx_venv/bin/python \
    client-server/graspgenx_server.py \
    --config /opt/graspgenx_checkpoints/release \
    --assets_dir /opt/graspgenx_gripper_descriptions/gripper_descriptions/assets \
    --default_gripper robotiq_2f_85 \
    --host 0.0.0.0 \
    --port 8782
```

---

### Terminal 3 — Robot State Publisher

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 launch ur5e_2f_85_moveit_config \
  isaac_robot_state_publisher.launch.py
```

### Terminal 4 — ros2_control

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

### Terminal 5 — Controllers + MoveIt

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run controller_manager spawner joint_state_broadcaster \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml

ros2 run controller_manager spawner hold_position_controller \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml

ros2 run controller_manager spawner scaled_joint_trajectory_controller \
  --inactive \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml

ros2 run controller_manager spawner robotiq_gripper_controller \
  --param-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml

ros2 launch ur5e_2f_85_moveit_config move_group_isaac.launch.py
```

### Terminal 6 — MoveIt Controller

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true \
  -p controller_to_stop:=hold_position_controller
```

### Terminal 7 — SeeDo Test

```bash
docker exec -it seedo_ros2_container bash

# The --config parameter is used to select the desired inference configuration

# FULL INFERENCE 
export SEEDO_ISAAC_GRASP_Z_OFFSET_M=0.020
export QT_QPA_PLATFORM=offscreen
bash /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/launch/run_seedo_isaac_test.sh

# PRECOMPUTED INFERENCE
export SEEDO_ISAAC_GRASP_Z_OFFSET_M=0.020
export QT_QPA_PLATFORM=offscreen
bash /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/launch/run_seedo_isaac_test.sh \
  --config /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/config/experiments/baseline_prior_guided_pick_place.yaml
  --precomputed-plan /test_real/test_task00_1/action_planning/action_plan.json \
  --precomputed-demo-scene /test_real/test_task00_1/demo_structured_scene/demo_structured_scene.json
```

The script can automatically configure the scene by asking for the **task** and **trajectory**.

To configure the scene manually before running the test:

```bash
python3 \
  /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/configure_isaac_scene.py
```
