# HOMESTRI-UR5e-Robotiq2f85

Docker file and docker-compose for ROS noetic  supporting packages for the UR5e and Robotiq 2F-85 gripper. The compose file starts two containers: the one running ROS and another running [noVNC](https://novnc.com/info.html) to use the GUI-based tools rviz and Gazebo.

### Installation
- Install either [docker engine](https://docs.docker.com/engine/install/ubuntu/) (Linux) or [Docker Desktop](https://www.docker.com/).
- Clone this repo and navigate to the parent directory.
- Enable X11 forwarding
    - Linux: `xhost +`
    - Windows: use [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
- Run `docker-compose up`.

### Enable GPU Acceleration (Tested on Ubuntu 22.04 with Nvidia GPU)
- Make sure latest Nvidia drivers are installed
- Install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- (Optional) Install [nvidia-cuda-toolkit](https://installati.one/install-nvidia-cuda-toolkit-ubuntu-22-04/)

### Calibrate the Real Robot
```
roslaunch homestri_bringup a_bot_calibration.launch
```

### Start the Real Robot (also starts Realsense camera and AprilTag tracking)
```
roslaunch homestri_bringup a_bot_bringup.launch
```

### Start the Simulated Robot in Gazebo
```
roslaunch homestri_bringup a_bot_gazebo.launch
```
###
### Start the realsense camera
- `roslaunch realsense2_camera rs_camera.launch`
- To ensure the camera is working, view the `/camera/color/image_raw topic` in RViz

### Start Aruco Tracking
- ensure `/camera_info` and `/image` topics are properly remapped in `aruco_ros/launch/double.launch`
- `roslaunch aruco_ros double.launch`

### Launch Teleop Cartesian Control
- Bringup the real robot
- Switch to velocity controller using `rqt_gui`
    - `rosrun rqt_gui rqt_gui`
    - In `rqt_gui` go to `Plugins > Robot Tools > Controller Manager`
    - Stop `scaled_pos_joint_traj_controller`
    - Run `joint_group_vel_controller`
- Launch joystick publisher
    - `roslaunch homestri_moveit_servo joystick_teleop.launch`
- Read and confirm joystick values and twist conversion
    - `rostopic echo /joy`
    - `rostopic echo /servo_server/cmd_vel`
- Launch `moveit_servo`
    - `roslaunch homestri_moveit_servo servo_server.launch`

### Execute Behavior Tree (With Aruco Tracking)
- Bringup the real robot (previous step)
- Start Aruco Tracking (previous step)
- Run manipulation_action_server
    - `rosrun homestri_manipulation manipulation_action_server.py`
- If you are grasping with an offset from a tag, use a static transform publisher to consider the offset (This will be changed later)
- Launch Explainable Behavior Tree UI
    - `rosrun rqt_gui rqt_gui`
    - In `rqt_gui` go to `Plugins > Actions > Explain BT Plugin`
- Execute behavior tree
    - `roslaunch homestri_behavior_trees my_bt.launch`