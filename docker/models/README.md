# SeeDo ROS2 Docker

## Build image

Run the following command from the repository root (`UR5e-2f-85`):

```bash
docker build \
    --build-arg BASE_IMAGE=ur_robotiq_teleoperation:latest \
    -f docker/models/SeeDo_Ros2 \
    -t seedo_ros2:latest \
    .
```

---

## Run container

```bash
docker run --rm -it \
    --name seedo_ros2_container \
    --gpus all \
    --network host \
    -v "$(pwd)":/home/ros2_ws/src/UR5e-2f-85 \
    -v "$(pwd)/../test_dataset":/test_dataset:ro \
    -v "$(pwd)/../seedo_tests":/seedo_tests \
    -v "$(pwd)/../scene_capture":/scene_capture:ro \
    -e OPENAI_API_KEY="$OPENAI_API_KEY" \
    -e BUILD_SEEDO_ROS=1 \
    seedo_ros2:latest
```

The entrypoint automatically:

- sources the ROS 2 environment;
- sources the workspace if available;
- configures the required `PYTHONPATH`;
- optionally builds the workspace if `BUILD_ROS_WS=1` is set.