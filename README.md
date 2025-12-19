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

## UR5e + Robotiq Gripper + Tavolo

**Build**
```
docker build -t ur_robotiq . -f UR_Robotiq 
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
  -v /home/mivia/Scrivania/Ur5e/ros2/ur_ros2/UR5e-2f-85/ur5e_2f_85:/home/ros2_ws/src/ur5e_2f_85 \
  ur_robotiq
``` 

```
colcon build --packages-select ur5e_2f_85_description ur5e_2f_85_moveit_config 
source install/setup.bash 
```


[X] Description file
[] Moveit Config

