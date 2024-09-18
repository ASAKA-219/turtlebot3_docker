FROM ubuntu:22.04
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# Timezone, Launguage設定
RUN apt update \
  && apt install -y --no-install-recommends \
     locales \
     software-properties-common tzdata \
  && locale-gen ja_JP ja_JP.UTF-8  \
  && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
  && add-apt-repository universe

# Locale
ENV LANG=ja_JP.UTF-8
ENV TZ=Asia/Tokyo

# packages install
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  g++ \
  iproute2 gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip \
  python3-tk \
  git wget curl unzip \
  x11-utils x11-apps terminator xterm xauth \
  xterm nano vim htop \
  build-essential software-properties-common gdb valgrind sudo \
  python3-venv lsb-release zlib1g

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME

RUN groupadd -g ${GID} ${GROUP_NAME} && \
    #groupadd -g ${GID} dialout && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} -G sudo ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/${USER_NAME}

# ROS2 install
RUN apt update \
  && apt install -y --no-install-recommends \
     curl gnupg2 lsb-release python3-pip vim wget build-essential ca-certificates git python3.10-venv
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update \
  && apt upgrade -y \
  && DEBIAN_FRONTEND=noninteractive \
  && apt install -y --no-install-recommends \
     ros-humble-desktop-full \
     git
RUN apt update && apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    ros-humble-smach-ros \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures
# pip install
RUN pip install setuptools==58.2.0 ;\
    pip install simpleaudio

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USER_NAME}/.bashrc ;\
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc ;\
    . /opt/ros/noetic/setup.bash ;\
    rosdep init

# create ws
RUN mkdir -p /home/${USER_NAME}/colcon_ws/src

### package setting is here
RUN apt update　;\
    apt install -y ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-dynamixel-sdk ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers ros-humble-moveit \
    ros-humble-moveit-servo ros-humble-gazebo-* ros-humble-realsense2-camera-msgs ros-humble-realsense2-description &&\
    usermod -aG dialout ${USER_NAME} &&\
# ps1
    echo "PS1='\[\033[48;5;10m\]TB3_LIME\[\033[0m\]@\[\033[32m\]\u\[\033[0m\]:\[\033[1;33m\]\w\[\033[0m\]$ '" >> /home/${USER_NAME}/.bashrc &&\
    chmod -R 777 /home/${USER_NAME}/colcon_ws

USER ${USER_NAME}

RUN cd /home/${USER_NAME}/colcon_ws/src ;\
    git clone -b humble-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_lime.git ;\
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git ;\
    git clone -b foxy-devel https://github.com/pal-robotics/realsense_gazebo_plugin.git &&\
    cd /home/${USER_NAME}/colcon_ws ;\
    . /opt/ros/humble/setup.bash ;\
    rosdep update ;\
    rosdep install -y -i --from-path src --rosdistro humble ;\
    colcon build --symlink-install ;\
    echo "source ~/colcon_ws/install/setup.bash" >> /home/${USER_NAME}/.bashrc &&\
    echo "source /usr/share/gazebo/setup.sh" >> /home/${USER_NAME}/.bashrc

# entrypoint
COPY assets/setup.sh /tmp/setup.sh
COPY assets/nanorc /home/${USER_NAME}/.nanorc
RUN sudo chmod +x /tmp/setup.sh
WORKDIR /home/${USER_NAME}/colcon_ws
ENTRYPOINT ["/tmp/setup.sh"]
