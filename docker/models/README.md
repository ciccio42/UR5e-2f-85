# SeeDo ROS2 Docker

## Build image

Run the following command from the repository root (`UR5e-2f-85`):

```bash
docker build \
    -f docker/models/SeeDo_Ros2 \
    -t seedo_ros2:latest \
    .
```

---

## Run container

```bash
docker run --rm -it \
    --gpus all \
    --network host \
    -v /home/mivia/Desktop/UR-Application/UR5e-2f-85:/home/ros2_ws/src/UR5e-2f-85 \
    -v /home/mivia/Desktop/UR-Application/test_dataset:/test_dataset:ro \
    -v /home/mivia/Desktop/UR-Application/seedo_tests:/seedo_tests \
    -e OPENAI_API_KEY=$OPENAI_API_KEY \
    seedo_ros2:latest
```

The entrypoint automatically:

- sources the ROS 2 environment;
- sources the workspace if available;
- configures the required `PYTHONPATH`;
- optionally builds the workspace if `BUILD_ROS_WS=1` is set.