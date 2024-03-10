FROM osrf/ros:noetic-desktop

# set environment variables
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# set work directory
RUN mkdir /workspace/src/mobile_robot_base_ros1 -p
WORKDIR /workspace

# copy project
# COPY . /workspace/src/tractor_mobile_platform/

# Additional commands (installing dependencies)
RUN apt-get update && \
    apt-get install -y \
    curl \
    wget \
    python3-pip

# Gazebo 11
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get install -y libgazebo11-dev 
# RUN sudo apt-get install -y ros-noetic-gazebo-ros
# RUN sudo apt-get install -y gazebo11

# RUN curl -sSL http://get.gazebosim.org | sh
RUN apt-get update && \
    apt-get install -y \
#     gazebo11 \
#     libgazebo11-dev 
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Other
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-control-toolbox \
    ros-noetic-realtime-tools \
    ros-noetic-ros-controllers \
    ros-noetic-xacro \
    python3-wstool \
    ros-noetic-tf-conversions \
    ros-noetic-kdl-parser \
    liburdfdom-tools

# Development tools
RUN apt-get update && \
    apt-get install -y \
    vim \
    git \
    tmux \
    tree 


# install python dependencies
COPY ../requirements.txt /tmp/requirements.txt
RUN pip install --upgrade pip
RUN pip install -r /tmp/requirements.txt

# added ros source
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "alias dros='source devel/setup.bash'" >> ~/.bashrc

# RUN rosdep init && rosdep update

RUN echo "All done!"
