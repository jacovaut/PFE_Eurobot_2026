FROM ros:jazzy-perception

# Arguments for user creation
ARG USERNAME=PFE
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Remove existing user with same UID if needed (safer with shell check)
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

# Update & install necessary packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip ros-jazzy-rqt* ros-jazzy-rviz2 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Set shell for user (not mandatory, but good for dev experience)
ENV SHELL=/bin/bash

# Switch to user
USER ${USERNAME}

# Start in bash by default
CMD ["/bin/bash"]
