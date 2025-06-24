# 1. Use ubuntu 24.04 base image
FROM ubuntu:24.04

# 2. Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# 3. Set locale to support UTF-8
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 

# 4. Add the ROS2 apt repository to the system
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    && add-apt-repository universe \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt install -y /tmp/ros2-apt-source.deb

# 5. Install ROS2 and development tools
RUN apt update && apt install -y ros-dev-tools \
    && apt update && apt upgrade -y && apt install -y ros-jazzy-desktop \
    # Clean up apt lists to reduce image size
    && rm -rf /var/lib/apt/lists/* 

# 6. Source the ROS2 setup script when a new shell is started
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 7. Start a Bash shell by default when the container runs
CMD ["/bin/bash"]