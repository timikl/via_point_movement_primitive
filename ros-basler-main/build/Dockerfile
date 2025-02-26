FROM ros:noetic
LABEL maintainer "Sebastian Ruiz <me@sruiz.uk>"

SHELL ["/bin/bash","-c"]

# ARG USER=docker
# ARG UID=1000
# ARG GID=1000

RUN apt update && apt install --no-install-recommends -y \
    git \
    vim \
    wget \
    curl \
    python3-catkin-tools \
    ros-noetic-image-proc \
    ros-noetic-image-pipeline \
    ros-noetic-image-transport-plugins \
    x11-apps

# RUN useradd -rm -d /home/$USER -s /bin/bash -g root -G sudo -u $UID $USER
# USER $USER

# Create local catkin workspace
# ENV CATKIN_WS=/home/$USER/catkin_ws
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

RUN cd $CATKIN_WS/src && git clone https://github.com/dragandbot/dragandbot_common.git

# Use our fork of pylon-ros-camera to remove image_rect topic being published
# RUN cd $CATKIN_WS/src && git clone https://github.com/basler/pylon-ros-camera.git
RUN cd $CATKIN_WS/src && git clone https://github.com/ReconCycle/pylon-ros-camera.git

# COPY pylon_colour_camera_node.launch $CATKIN_WS/src/pylon-ros-camera/pylon_camera/launch/
# COPY colour_camera.yaml $CATKIN_WS/src/pylon-ros-camera/pylon_camera/config/

# COPY calibration_1450x1450_goe.yaml $CATKIN_WS/src/pylon-ros-camera/pylon_camera/config/
# COPY calibration_1450x1450_jsi.yaml $CATKIN_WS/src/pylon-ros-camera/pylon_camera/config/

RUN sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list'

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y

# catkin build
RUN source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    catkin init && \
    catkin clean -y && \
    catkin build

# Always source ros_catkin_entrypoint.sh when launching bash (e.g. when attaching to container)
RUN echo "source /entrypoint.sh" >> /root/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# gosu
RUN set -eux; \
	apt-get update; \
	apt-get install -y gosu; \
	rm -rf /var/lib/apt/lists/*; \
# verify that the binary works
	gosu nobody true

VOLUME /root/catkin_ws/src/pylon-ros-camera/pylon_camera/config

ENTRYPOINT ["/entrypoint.sh"]

# stop docker from exiting immediately
CMD tail -f /dev/null