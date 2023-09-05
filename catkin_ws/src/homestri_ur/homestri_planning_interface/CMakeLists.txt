cmake_minimum_required(VERSION 3.0.2)
project(homestri_planning_interface)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    message_generation
    std_msgs
    geometry_msgs
    trajectory_msgs
)

catkin_python_setup()

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SimplePlan.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    trajectory_msgs
)

catkin_install_python(
    PROGRAMS 
        scripts/planning_interface
        scripts/test_compliant_traj
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
