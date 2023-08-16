FROM iantc104/cudagl-ros-noetic:1.0

SHELL ["/bin/bash", "-c"]

# Install general dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-catkin-tools 

# Install python dependencies
RUN pip3 install \ 
    pyserial \
    pymodbus===2.1.0

# Install ROS dependencies
RUN apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description \
    ros-noetic-rqt-controller-manager

# Set the working directory in the container
WORKDIR /root/catkin_ws

# Copy the 'src' directory from 'catkin_ws' to the container's workspace
COPY ./catkin_ws/src ./src

# Install ROS package dependencies
RUN rosdep update && \
    rosdep install -y --from-paths ./src --ignore-src --rosdistro noetic

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin build

# Source the workspace setup files on container startup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc