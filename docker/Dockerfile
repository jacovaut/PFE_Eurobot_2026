FROM ros:jazzy-ros-base

ARG USERNAME=piros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Remove existing user with same UID if needed
RUN if id -u ${USER_UID} 2>/dev/null; then \
        OLDUSER=$(id -un ${USER_UID}) && userdel -r ${OLDUSER}; \
    fi

# Create user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Install required packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        python3-opencv \
        python3-numpy \
        python3-transforms3d \
        python3-scipy \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2 \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-visualization-msgs \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-rviz2 \
        ros-jazzy-robot-localization \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
   
ENV ROS_DOMAIN_ID=0
ENV SHELL=/bin/bash

USER ${USERNAME}

RUN echo '' >> /home/${USERNAME}/.bashrc && \
    echo '# ROS 2 setup' >> /home/${USERNAME}/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /home/${USERNAME}/.bashrc && \
    echo "export USERNAME=${USERNAME}" >> /home/${USERNAME}/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /home/${USERNAME}/PFE_Eurobot_2026/robot_ws/install/setup.bash ]; then" >> /home/${USERNAME}/.bashrc && \
    echo "    source /home/${USERNAME}/PFE_Eurobot_2026/robot_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo 'fi' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /home/${USERNAME}/PFE_Eurobot_2026/server_ws/install/setup.bash ]; then" >> /home/${USERNAME}/.bashrc && \
    echo "    source /home/${USERNAME}/PFE_Eurobot_2026/server_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo 'fi' >> /home/${USERNAME}/.bashrc && \
    echo '' >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /home/${USERNAME}/PFE_Eurobot_2026/uros_ws/install/setup.bash ]; then" >> /home/${USERNAME}/.bashrc && \
    echo "    source /home/${USERNAME}/PFE_Eurobot_2026/uros_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo 'fi' >> /home/${USERNAME}/.bashrc

CMD ["bash"]