# Dockerfile

FROM ubuntu:22.04

# Setup locale
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
    curl wget git lsb-release gnupg2 sudo locales \
    x11-apps libgl1-mesa-glx libxrender1 libxi6 \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8  
ENV LC_ALL en_US.UTF-8

# Install ROS 2 Humble
RUN apt update && apt install -y software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' \
    && apt update \
    && apt install -y ros-humble-desktop python3-colcon-common-extensions

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Webots
RUN wget https://cyberbotics.com/ephemeral/webots_2024-a_amd64.deb \
    && apt install -y ./webots_2024-a_amd64.deb \
    && rm webots_2024-a_amd64.deb

# Install Webots ROS2 interface
RUN git clone https://github.com/cyberbotics/webots_ros2.git /root/webots_ros2 \
    && cd /root/webots_ros2 && rosdep update && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build

WORKDIR /root/webots_ros2
CMD ["/bin/bash"]
