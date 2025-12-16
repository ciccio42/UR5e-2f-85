# Docker for UR-ROS2

## ROS2 docker

**Build**
``` 
docker build -t ros2 . -f ros2Rolling
``` 

**Run**
``` 
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ros2
``` 


## UR

**Build**
``` 
docker build -t ur_ros2 . -f URRos2
``` 


**Run**
```
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  ur_ros2
``` 

## UR5e + Robotiq Gripper

**Build**
``` 
``` 


**Run**
```
``` 