FROM osrf/ros:noetic-desktop as base
WORKDIR /catkin_ws/src
RUN apt update && apt install -q -y \
    ros-noetic-rosserial-arduino && \
    rm -rf /var/lib/apt/lists/*
RUN bash -c "source /ros_entrypoint.sh && \
    catkin_init_workspace && \
    cd .. && catkin_make"
WORKDIR /catkin_ws
ENTRYPOINT [ "/ros_entrypoint.sh" ]