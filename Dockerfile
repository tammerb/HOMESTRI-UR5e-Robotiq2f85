FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt update -qq && apt install -y \
git python3-pip ros-noetic-moveit \
&& pip3 install catkin_tools pyserial pymodbus===2.1.0

# Setup environment, all installed things go here
WORKDIR /catkin_ws

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

ENV MESH_WORKSPACE_PATH='/catkin_ws/src'
ENV GAZEBO_MODEL_PATH='/catkin_ws/src'

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin build

COPY docker-entrypoint.sh .
CMD "source /catkin_ws/docker-entrypoint.sh"