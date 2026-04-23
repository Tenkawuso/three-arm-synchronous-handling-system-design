FROM docker.m.daocloud.io/osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-serial \
    python3-opencv \
    python3-numpy \
    ros-noetic-moveit \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
