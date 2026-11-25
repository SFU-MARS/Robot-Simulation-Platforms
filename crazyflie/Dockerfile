FROM ros:humble

# 1. Desktop tools, Foxglove bridge and basic utilities
RUN apt-get update && apt-get install -y \
    ros-humble-desktop ros-humble-foxglove-bridge \
    nano git sudo swig \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. Crazyswarm2 system dependencies
RUN apt-get update && apt-get install -y \
    libboost-program-options-dev libusb-1.0-0-dev \
    ros-humble-motion-capture-tracking \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# 3. Crazyswarm2 Python dependencies
RUN pip3 install "numpy<2" "setuptools>=62.4,<70" rowan nicegui cflib \
    && pip3 install --upgrade transforms3d

# 4. Create a non-root user with sudo
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 5. Source ROS and the workspace automatically in every terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "[ -f ~/ros2ws/install/setup.bash ] && source ~/ros2ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# 6. Start as the ros user, inside the workspace
USER $USERNAME
WORKDIR /home/$USERNAME/ros2ws
ENV PYTHONPATH=/home/ros/ros2ws/crazyflie-firmware/build