cmake_minimum_required(VERSION 3.0.2)
project(homestri_behavior_trees)

set(ROS_DEPENDENCIES
roscpp
behaviortree_cpp_v3
behaviortree_ros
std_msgs
actionlib
actionlib_msgs
tf2
tf2_ros
homestri_msgs
)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES
    # CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(test_bt src/test_bt.cpp)
add_dependencies(test_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bt ${catkin_LIBRARIES} )