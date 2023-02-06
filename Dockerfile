FROM nvidia/cudagl:11.1.1-base-ubuntu20.04

SHELL ["/bin/bash", "-c"]

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install dependencies
RUN apt install -y git python3-pip ros-noetic-moveit \
&& pip3 install catkin_tools pyserial pymodbus===2.1.0

# Setup environment, all installed things go here
WORKDIR /catkin_ws

# Install ROS packages
RUN git clone https://github.com/tammerb/HOMESTRI-UR5e-Robotiq2f85.git src/homestri \
 && git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/universal_robots/Universal_Robots_Client_Library \
 && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/universal_robots/Universal_Robots_ROS_Driver \
 && git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robots/universal_robot \
 && git clone -b noetic-devel https://github.com/ian-chuang/robotiq.git src/robotiq \
 && git clone https://github.com/ian-chuang/roboticsgroup_upatras_gazebo_plugins.git src/roboticsgroup_upatras_gazebo_plugins \
 && git clone https://github.com/ian-chuang/gazebo_gripper_action_controller.git src/gazebo_gripper_action_controller \
 && git clone https://github.com/ian-chuang/BehaviorTree.ROS.git src/behaviortree_ros \ 
 && git clone https://github.com/ian-chuang/BehaviorTree.CPP.git src/behaviortree_cpp_v3 \
 && git clone https://github.com/ian-chuang/robot-explanation-BTs.git src/explain_bt

# Install ROS package dependencies
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin build

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc