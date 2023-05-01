FROM thbarkouki/nvidia-cuda-noetic:latest

SHELL ["/bin/bash", "-c"]

# Setup environment, all installed things go here
WORKDIR /catkin_ws

RUN apt-get update

# Install dependencies
RUN apt-get install -y ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-rqt-controller-manager

# Install ROS packages
RUN git clone https://github.com/tammerb/HOMESTRI-UR5e-Robotiq2f85.git src/homestri \ 
 && git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/universal_robots/Universal_Robots_Client_Library \
 && git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/universal_robots/Universal_Robots_ROS_Driver \
 && git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robots/universal_robot \
 && git clone -b noetic-devel https://github.com/ian-chuang/robotiq.git src/robotiq \
 && git clone https://github.com/ian-chuang/OnRobot_RG2-FT_ROS.git src/onrobot \
 && git clone https://github.com/ian-chuang/roboticsgroup_upatras_gazebo_plugins.git src/gazebo_pkgs/roboticsgroup_upatras_gazebo_plugins \
 && git clone https://github.com/ian-chuang/gazebo_gripper_action_controller.git src/gazebo_pkgs/gazebo_gripper_action_controller \
 && git clone https://github.com/ian-chuang/BehaviorTree.ROS.git src/behavior_trees/behaviortree_ros \ 
 && git clone -b v3.8 https://github.com/ian-chuang/BehaviorTree.CPP.git src/behavior_trees/behaviortree_cpp_v3 \
 && git clone https://github.com/ian-chuang/robot-explanation-BTs.git src/behavior_trees/explain_bt \ 
 && git clone https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers src/cartesian_controllers \
 && git clone -b noetic-devel https://github.com/pal-robotics/aruco_ros src/aruco_ros 

# Install ROS package dependencies
RUN source /opt/ros/noetic/setup.bash \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin build

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc