FROM ros:jazzy-ros-base

ARG USERNAME=ubuntu

# Install some dependencies packages
RUN apt update -q \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip\
    xauth \
    ninja-build \
    ccache \
    clangd-19 \
    gdb \
    bash-completion \
    pre-commit \
    nano \
    && apt clean \
    && ln -s /usr/bin/clangd-19 /usr/bin/clangd \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN  echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc \
    && rosdep update \
    && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true \
    && colcon mixin update
# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
