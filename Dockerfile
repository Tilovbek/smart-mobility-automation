# TurtleBot3 Automation Suite - Docker

FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Foxy
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-turtlebot3* \
    ros-foxy-nav2* \
    ros-foxy-slam-toolbox \
    ros-foxy-vision-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy project files
COPY . /workspace/src/

# Install Python dependencies
RUN pip3 install -r src/requirements.txt

# Build the project
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# Set up environment
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]