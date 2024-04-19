# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ros:noetic AS core

RUN apt-get update && apt-get install -q -y \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

