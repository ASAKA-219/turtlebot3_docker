FROM osrf/ros:noetic-desktop
LABEL maintainer="Asaka Yusuke <yusuke.asaka@gmail.com>"
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

# ROS noetic install
RUN apt update &&\
    apt install -y python3-catkin-tools python3-rosdep \
    ros-noetic-turtlebot3* ros-noetic-gmapping ros-noetic-gazebo-ros-pkgs &&\
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init

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

USER ${USER_NAME}

# create ws
RUN mkdir -p /home/${USER_NAME}/catkin_ws/src && pip install setuptools==58.2.0 &&\
    chmod -R 777 /home/${USER_NAME}/catkin_ws &&\
    echo "source /opt/ros/noetic/setup.bash" >> /home/${USER_NAME}/.bashrc &&\
    source /home/${USER_NAME}/.bashrc &&\
    cd /home/${USER_NAME}/catkin_ws ;\
    rosdep update ; rosdep install -iy --from-paths src

RUN cd /home/${USER_NAME}/catkin_ws/src/ ;\
    source /opt/ros/noetic/setup.bash ;\
    git clone -b noetic-jp-devel https://github.com/ROBOTIS-JAPAN-GIT/turtlbot3_simulations.git ;\
    echo "export PS1='\[\033[44;37m\]NOETIC\[\033[0m\]:\[\033[32m\]\u\[\033[0m\]:\[\033[1;33m\]\w\[\033[0m\]$ '" >> /home/${USER_NAME}/.bashrc &&\
    echo 'export TURTLEBOT3_PLAT=false' >> /home/${USER_NAME}/.bashrc ;\
    echo 'export LDS_MODEL=LDS-02' >> /home/${USER_NAME}/.bashrc ;\
    echo 'export TURTLEBOT3_MODEL=burger' >> /home/${USER_NAME}/.bashrc
    
COPY tb3_common /home/${USER_NAME}/catkin_ws/src/tb3_common
RUN sudo apt update && sudo apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 &&\
    
    cd /home/${USER_NAME}/catkin_ws ;\
    source /opt/ros/noetic/setup.bash ;\
    catkin build 

# entrypoint
COPY assets/setup.sh /tmp/entry_point.sh
COPY assets/nanorc /home/${USER_NAME}/.nanorc
RUN echo 'source ~/catkin_ws/devel/setup.bash' >> /home/${USER_NAME}/.bashrc ;\
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> /home/${USER_NAME}/.bashrc
WORKDIR /home/${USER_NAME}/catkin_ws
ENTRYPOINT ["/tmp/entry_point.sh"]
