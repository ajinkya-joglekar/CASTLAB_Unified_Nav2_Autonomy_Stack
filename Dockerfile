FROM osrf/ros:jazzy-desktop-noble

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV GZ_VERSION=harmonic

# Add Gazebo repository
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg | \
      gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    gedit \
    mesa-utils \
    python3-pip \
    python3-yaml \
    python3-opencv \
    python3-numpy \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-nmea-navsat-driver \
    ros-jazzy-geodesy \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-mapviz \
    ros-jazzy-mapviz-plugins \
    ros-jazzy-tile-map \
    ros-jazzy-depth-image-proc \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY ws/ /ws/

RUN pip install --break-system-packages "onnxruntime>=1.10.0"

# Initial build
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select \
      base_nav2_stack \
      semantic_segmentation_node \
      terrain_obstacle_mapper \
      terrain_costmap_layer

# Use the same interactive setup as bind-mounted development containers.
RUN cat /ws/.bashrc_nav2 >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]
