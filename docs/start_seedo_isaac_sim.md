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
/isaac-sim/usd/ur5e_2f_85/seedo_scene_v2.usd
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
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/home/ros2_ws/src/UR5e-2f-85 \
  -v "$(pwd)/../test_dataset":/test_dataset:ro \
  -v "$(pwd)/../seedo_tests":/seedo_tests \
  -v "$(pwd)/../scene_capture":/scene_capture:ro \
  -v "$(pwd)/../test_isaac":/test_isaac \
  -v "$(pwd)/../test_real":/test_real \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e OPENAI_API_KEY="$OPENAI_API_KEY" \
  -e BUILD_SEEDO_ROS=1 \
  -e ROS_DOMAIN_ID=42 \
  seedo_ros2:latest
```

### Terminal 1 — Robot State Publisher

```bash
source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 launch ur5e_2f_85_moveit_config \
  isaac_robot_state_publisher.launch.py
```

### Terminal 2 — ros2_control

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file /home/ros2_ws/src/UR5e-2f-85/ur5e_2f_85/ur5e_2f_85_moveit_config/config/ros2_controllers_isaac.yaml
```

### Terminal 3 — Controllers + MoveIt

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

### Terminal 4 — MoveIt Controller

```bash
docker exec -it seedo_ros2_container bash

source /home/ros2_ws/install/setup.bash
source /opt/topic_based_ws/install/setup.bash

ros2 run moveit_controller moveit_controller_node \
  --ros-args \
  -p execute_trajectory:=true \
  -p controller_to_stop:=hold_position_controller
```

### Terminal 5 — SeeDo Test

```bash
docker exec -it seedo_ros2_container bash

bash /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/run_seedo_isaac_test.sh
```

The script can automatically configure the scene by asking for the **task** and **trajectory**.

To configure the scene manually before running the test:

```bash
python3 \
  /home/ros2_ws/src/UR5e-2f-85/ai_controller/ai_controller/models/seedo_controller/configure_isaac_scene.py
```
