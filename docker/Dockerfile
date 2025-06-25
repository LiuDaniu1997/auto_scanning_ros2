FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y apt-utils

# Install necessary programs
RUN apt-get install -y \
    nano \
    vim \
    git \
    curl \
    lsb-release \
    wget \
    gnupg \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=robot
ARG USER_UID=1010
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Install gz-harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install pcl
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install yaml
RUN apt-get update && apt-get install -y \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install gz-msgs
RUN apt-get update && apt-get install -y \
    libgz-msgs11-dev \
    && rm -rf /var/lib/apt/lists/*

# Install gz-transport
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y libgz-transport14-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ros-jazzy-ros-gz and its dependencies
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Install ros-jazzy-topic tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-topic-tools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

####################################################################################################
# Set up ROS 2 workspace
USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p deburring_robot_ws/src
RUN mkdir -p deburring_robot_ws/data/temp
RUN mkdir -p deburring_robot_ws/data/merged_point_cloud
RUN mkdir -p deburring_robot_ws/data/aligned_point_cloud
RUN mkdir -p deburring_robot_ws/data/img

COPY src/ deburring_robot_ws/src/

WORKDIR /home/$USERNAME/deburring_robot_ws

# Install ros dependencies
RUN sudo apt-get update && sudo apt-get install -y python3-colcon-common-extensions python3-rosdep \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y

RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

