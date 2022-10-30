FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt update -qq && apt install -y \
git python3-pip ros-noetic-moveit \
&& pip3 install catkin_tools pyserial pymodbus

# Setup environment, all installed things go here
WORKDIR /catkin_ws
RUN git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/Universal_Robots_Client_Library \
&& git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver \
&& git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robot \
&& git clone https://github.com/jr-robotics/robotiq src/robotiq \
&& git clone https://github.com/Pitrified/gazebo_ros_link_attacher.git src/gazebo_ros_link_attacher \
&& git clone https://github.com/andreasBihlmaier/gazebo2rviz.git src/gazebo2rviz \
&& git clone https://github.com/andreasBihlmaier/pysdf.git src/pysdf \
&& git clone https://github.com/tammerb/HOMESTRI-UR5e-Robotiq2f85.git src \ 
ENV MESH_WORKSPACE_PATH='/catkin_ws/src'
ENV GAZEBO_MODEL_PATH='/catkin_ws/src'

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin build

COPY docker-entrypoint.sh .
CMD "source /catkin_ws/docker-entrypoint.sh"