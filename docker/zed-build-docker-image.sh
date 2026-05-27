#!/bin/bash

ZED_SDK_MAJOR=5 		# ZED SDK major version 
ZED_SDK_MINOR=3 		# ZED SDK minor version
L4T_RELEASE=r38.4 			# L4T release for ARM64 ZED image
ROS_DISTRO_ARG="jazzy"
TARGET_PLATFORM="${TARGET_PLATFORM:-linux/arm64}"

echo "ROS2 flag == 1 " 
TAG="${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}-ros2-devel-l4t-${L4T_RELEASE}"
DOCKERFILE="ZedRos2"

echo "Building '${TAG}'" 
echo "ROS2 Distro: ${ROS_DISTRO_ARG}"
echo "Target platform: ${TARGET_PLATFORM}"

docker build \
    --platform "${TARGET_PLATFORM}" \
    --build-arg ZED_SDK_MAJOR=${ZED_SDK_MAJOR} \
    --build-arg ZED_SDK_MINOR=${ZED_SDK_MINOR} \
    --build-arg ROS_DISTRO_ARG=${ROS_DISTRO_ARG} \
    --build-arg TARGET_PLATFORM=${TARGET_PLATFORM} \
    -t "${TAG}" . -f "${DOCKERFILE}"

