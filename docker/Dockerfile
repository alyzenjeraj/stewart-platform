FROM ros:humble AS base

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO=humble

# Set the default working directory for the ROS2 workspace
WORKDIR /ros2_ws

# Install dependencies and build the workspace
# RUN apt-get update && rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN colcon build

# RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]