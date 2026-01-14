#!/bin/bash

UBUNTU_RELEASE_YEAR=24 	#Specify the Ubunutu release year
ZED_SDK_MAJOR=5 		# ZED SDK major version 
ZED_SDK_MINOR=1 		# ZED SDK minor version
CUDA_MAJOR=13 			# CUDA major version
CUDA_MINOR=0 			# CUDA minor version 
ROS_DISTRO_ARG="jazzy"

echo "ROS2 flag == 1 " 
TAG="${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}-ros2-devel-cuda${CUDA_MAJOR}.${CUDA_MINOR}-ubuntu${UBUNTU_RELEASE_YEAR}.04"
DOCKERFILE="ZedRos2"

echo "Building '${TAG}'" 
echo "ROS2 Distro: ${ROS_DISTRO_ARG}"

docker build \
    --build-arg UBUNTU_RELEASE_YEAR=${UBUNTU_RELEASE_YEAR} \
    --build-arg ZED_SDK_MAJOR=${ZED_SDK_MAJOR} \
    --build-arg ZED_SDK_MINOR=${ZED_SDK_MINOR} \
    --build-arg CUDA_MAJOR=${CUDA_MAJOR} \
    --build-arg CUDA_MINOR=${CUDA_MINOR} \
    --build-arg ROS_DISTRO_ARG=${ROS_DISTRO_ARG} \
    -t "${TAG}" . -f "${DOCKERFILE}"

