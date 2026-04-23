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
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-robot-localization \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=0
ENV SHELL=/bin/bash

USER ${USERNAME}

RUN cat >> /home/${USERNAME}/.bashrc << EOF

# ROS 2 setup
export ROS_DOMAIN_ID=0
export USERNAME=${USERNAME}
source /opt/ros/jazzy/setup.bash

if [ -f /home/${USERNAME}/PFE_Eurobot_2026/robot_ws/install/setup.bash ]; then
    source /home/${USERNAME}/PFE_Eurobot_2026/robot_ws/install/setup.bash
fi

if [ -f /home/${USERNAME}/PFE_Eurobot_2026/server_ws/install/setup.bash ]; then
    source /home/${USERNAME}/PFE_Eurobot_2026/server_ws/install/setup.bash
fi

if [ -f /home/${USERNAME}/PFE_Eurobot_2026/uros_ws/install/setup.bash ]; then
    source /home/${USERNAME}/PFE_Eurobot_2026/uros_ws/install/setup.bash
fi
EOF

CMD ["bash"]