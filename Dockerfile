FROM nvidia/cuda:11.1.1-cudnn8-devel-ubuntu20.04
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
  iproute2 gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip \
  python3-tk \
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

# ROS noetic install
RUN echo "debug"
RUN DEBIAN_FRONTEND=noninteractive ;\
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' ;\
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - ;\
    sudo apt update ;\
    sudo apt install -y ros-noetic-desktop \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    python3-osrf-pycommon python3-catkin-tools;\
    echo "source /opt/ros/noetic/setup.bash" >> /home/${USER_NAME}/.bashrc
RUN sudo rosdep init

#RUN apt update && apt install -y \
    #python3-flake8-docstrings \
#    python3-pip \
#    python3-pytest-cov \
#    ros-dev-tools \
#    ros-noetic-smach-ros \
    #python3-pytest-repeat \
    #python3-pytest-rerunfailures
# pip install
#RUN pip install setuptools==58.2.0
#RUN pip install simpleaudio
RUN apt update && apt install -y ros-noetic-smach-ros ros-dev-tools ros-noetic-joy \
    ros-noetic-turtlebot3* ros-noetic-gmapping ros-noetic-gazebo-ros-pkgs

# create ws
RUN mkdir -p /home/${USER_NAME}/catkin_ws/src

### package setting is here
RUN sudo apt update; sudo apt install -y 
###

# user setting
RUN usermod -aG dialout ${USER_NAME}
# ps1
RUN echo "PS1='\[\033[44;37m\]NOETIC\[\033[0m\]@\[\033[32m\]\u\[\033[0m\]:\[\033[1;33m\]\w\[\033[0m\]$ '" >> /home/${USER_NAME}/.bashrc
# build

RUN chmod -R 777 /home/${USER_NAME}/catkin_ws
USER ${USER_NAME}
RUN cd /home/${USER_NAME}/catkin_ws/src/ ;\
    source /opt/ros/noetic/setup.bash ;\
    git clone -b noetic-jp-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_simulations.git ;\
    cd ../ && catkin build ;\
    echo 'source ~/catkin_ws/devel/setup.bash' >> /home/${USER_NAME}/.bashrc ;\
    echo 'export TURTLEBOT3_PLAT=false' >> /home/${USER_NAME}/.bashrc ;\
    echo 'export LDS_MODEL=LDS-01' >> /home/${USER_NAME}/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=burger' >> /home/${USER_NAME}/.bashrc ;\
    rosdep update ;\
    rosdep install -y -i --from-paths /home/${USER_NAME}/catkin_ws/src/ --ignore-src --rosdistro noetic

COPY tb3_common /home/${USER_NAME}/catkin_ws/src/tb3_common
COPY tb3_navigation /home/${USER_NAME}/catkin_ws/src/tb3_navigation
RUN cd catkin_ws ; catkin build
# entrypoint
COPY assets/setup.sh /tmp/setup.sh
COPY assets/nanorc /home/${USER_NAME}/.nanorc
RUN sudo chmod +x /tmp/setup.sh ; echo 'export ROS_MASTER_URI=http://yusuke:11311' >> /home/${USER_NAME}/.bashrc
WORKDIR /home/${USER_NAME}/catkin_ws
ENTRYPOINT ["/tmp/setup.sh"]
