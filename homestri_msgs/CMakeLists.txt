cmake_minimum_required(VERSION 3.0.2)
project(homestri_msgs)

find_package(catkin REQUIRED
std_msgs
geometry_msgs
message_generation
actionlib_msgs
actionlib
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
# )

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Grasps.srv
# )

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Manipulation.action
  ExecuteBehaviorTree.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  actionlib_msgs
)

catkin_package()

include_directories()

