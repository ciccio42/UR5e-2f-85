# ZED Cameras

```bash
# Build docker
source zed-build-docker-image.sh

xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --ipc=host \
  --pid=host \
  --device=/dev/bus/usb \
  -v /sys:/sys:ro \
  -v /run/udev:/run/udev:ro \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v ${UR5e_2f_85_PATH}/zed_camera:/home/ros2_ws/src/zed_camera \
  --name zed_camera_container \
  5.3-ros2-devel-l4t-r38.4

# Test camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
# RVIZ
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedm

# Aruco detection
ros2 launch zed_camera_calibration zed_camera_calibration.launch.py \
            camera_model:=zedm \
            config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml

# Launch multi-camera 
#ros2 launch zed_multi_camera zed_multi_camera.launch.py \
#      cam_names:='[zed_front,zed_left,zed_right]' \
#      cam_models:='[zedm,zedm,zedm,zedm]' \
#      cam_serials:='[16450494,11990492,15689351,16702584]'

ros2 launch zed_camera_calibration zed_multi_camera_calibration.launch.py \
            camera_model:=zedm \
            config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml \
            rviz:=false

# Run interactive calibration
ros2 launch zed_camera_driver zed_multi_camera.launch.py \
    camera_model:='zedm' \
    config_camera_path:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml \
    cameras_yaml:=src/zed_camera/zed_camera_calibration/config/multi_cameras.yaml \
    rviz:=true

ros2 run zed_camera_calibration interactive_aruco_calibration.py \
    --ros-args \
    -p cameras_config:=src/zed_camera/zed_camera_calibration/config/multi_cameras.yaml \
    -p cameras_yaml:=src/zed_camera/zed_camera_calibration/config/camera_config.yaml \
    -p aruco_info:=src/zed_camera/zed_camera_calibration/config/aruco_frontal_camera.yaml

ros2 run zed_camera_calibration interactive_aruco_calibration_until_pose.py \
  --ros-args \
  -p cameras_config:=src/zed_camera/zed_camera_calibration/config/exig.yaml \
  -p aruco_info:=src/zed_camera/zed_camera_calibration/config/aruco_frontal_camera.yaml

```