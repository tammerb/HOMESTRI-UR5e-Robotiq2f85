 
#!/bin/bash

ECHO_PREFIX="[docker-entrypoint.sh]"

# location of master
# export ROS_MASTER_URI=http://nils:11311/
# echo "$ECHO_PREFIX" "set ROS master: " "$ROS_MASTER_URI"

# ROS installation
ROS=/opt/ros/noetic/setup.bash
source "$ROS"
echo "$ECHO_PREFIX" "sourced ROS installation:" "$ROS"

# workspace holding custom ROS packages
workspace=/catkin_ws

source "$workspace"/devel/setup.bash
echo "$ECHO_PREFIX" "sourced workspace:" "$workspace"

echo "$ECHO_PREFIX" "call" "$@"
exec "$@"