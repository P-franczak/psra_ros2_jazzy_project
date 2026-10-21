FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

ENV RCUTILS_COLORIZED_OUTPUT=1
# Ubuntu 24.04 (PEP 668) blocks system-wide pip installs without this flag
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt update && \
    apt install -y \
        build-essential \
        cmake \
        git \
        curl lsb-release gnupg \
        mesa-utils \
        libbullet-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-colcon-clean \
        wget \
        clang-format \
        python3-argcomplete \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-docstrings \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest \
        python3-pytest-repeat \
        python3-pytest-rerunfailures \
        python3-jsonschema \
        python3-jinja2

RUN pip3 install \
    kconfiglib \
    pyros-genmsg \
    future


# Gazebo Harmonic is the default Gazebo version for ROS 2 Jazzy - no OSRF repo needed
RUN apt-get update -y && \
    apt-get install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-turtle-tf2-py \
        ros-jazzy-tf2-tools

RUN apt-get update -y
RUN apt-get upgrade -y

# Ubuntu 24.04 ships an 'ubuntu' user with UID 1000 - remove it to free that ID.
# devcontainer.json ("updateRemoteUserUID") remaps developer to the host user's UID/GID,
# so files in the bind-mounted workspace stay writable.
RUN userdel -r ubuntu 2>/dev/null || true && \
    groupadd -g 1000 developer && \
    useradd -m -d /home/developer -s /bin/bash -u 1000 -g 1000 -G sudo developer && \
    echo 'developer:developer' | chpasswd && \
    echo 'developer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer
USER developer

# RUN git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --recursive -b main && \
#     echo developer | sudo -S bash /home/developer/PX4-Autopilot/Tools/setup/ubuntu.sh

COPY --chown=developer:developer . /home/developer/ros2_ws/
WORKDIR /home/developer/ros2_ws


RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo ". /home/developer/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD [ "sleep", "infinity" ]