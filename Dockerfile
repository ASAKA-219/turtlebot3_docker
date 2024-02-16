FROM ubuntu:16.04
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
ENV LANG ja_JP.UTF-8
ENV TZ=Asia/Tokyo

# keyboard setting
RUN apt install -y  -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" keyboard-configuration

# basic packages install
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  g++ \
  iproute2 gnupg gnupg2 \
  libcanberra-gtk* \
  python-pip \
  python-tk \
  git wget curl unzip \
  x11-utils x11-apps terminator xauth \
  xterm nano vim htop \
  software-properties-common gdb valgrind sudo \
  python3-venv lsb-release zlib1g

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME
ARG PASSWORD
ARG PKGS_PATH_RLT

RUN groupadd -g ${GID} ${GROUP_NAME} && \
    useradd -m -s /bin/bash -u ${UID} -g ${GID} -G sudo ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/${USER_NAME}

# ROS kinetic install
RUN echo "debug"
RUN DEBIAN_FRONTEND=noninteractive ;\
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' ;\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - ;\
    sudo apt update ;\
    sudo apt install -y ros-kinetic-desktop \
    python-rosdep python-rosinstall python-rosinstall-generator python-vcstools build-essential \
    python-osrf-pycommon python-catkin-tools;\
    echo "source /opt/ros/kinetic/setup.bash" >> /home/${USER_NAME}/.bashrc
RUN sudo rosdep init ; rosdep update

RUN apt update && apt install -y  ros-kinetic-joy ros-kinetic-dynamixel-sdk ros-kinetic-ros-control* ros-kinetic-control* ros-kinetic-moveit*\
    ros-kinetic-turtlebot3* ros-kinetic-gmapping ros-kinetic-gazebo-ros-* ros-kinetic-ar-track-alvar ros-kinetic-ar-track-alvar-msgs ros-kinetic-joint-state-publisher-gui

# create ws
RUN mkdir -p /home/${USER_NAME}/catkin_ws/src 

# user setting
RUN usermod -aG dialout ${USER_NAME}
# ps1
RUN echo "PS1='\[\033[44;37m\]KINETIC\[\033[0m\]:\[\033[32m\]\u\[\033[0m\]:\[\033[1;33m\]\w\[\033[0m\]$ '" >> /home/${USER_NAME}/.bashrc

# build
RUN chmod -R 777 /home/${USER_NAME}/catkin_ws
USER ${USER_NAME}

RUN cd /home/${USER_NAME}/catkin_ws/src/ ;\
    source /opt/ros/kinetic/setup.bash
    
# entrypoint
COPY assets/setup.sh /tmp/setup.sh
COPY assets/nanorc /home/${USER_NAME}/.nanorc
RUN sudo chmod +x /tmp/setup.sh ;\
    echo 'source ~/catkin_ws/devel/setup.bash' >> /home/${USER_NAME}/.bashrc
    #echo 'export ROS_MASTER_URI=http://yusuke:11311' >> /home/${USER_NAME}/.bashrc
WORKDIR /home/${USER_NAME}/catkin_ws
ENTRYPOINT ["/tmp/setup.sh"]
