cmake_minimum_required(VERSION 3.0.2)
project(homestri_manipulation)


find_package(catkin REQUIRED
rospy
roscpp
std_msgs
geometry_msgs
moveit_msgs
actionlib
actionlib_msgs
tf
moveit_core
moveit_visual_tools
moveit_ros_planning
moveit_ros_planning_interface
homestri_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shr_manipulation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


