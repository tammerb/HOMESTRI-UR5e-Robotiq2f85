# HOMESTRI-UR

Welcome to the HOMESTRI-UR repository! This repository provides a Dockerized workspace tailored for working with UR5e robot arms at the HRVIP Lab and HOME Space Technology Research Institute. The Docker container is preconfigured with all the necessary dependencies and settings, streamlining the process of developing and experimenting with UR5e robots within a consistent and isolated environment.

## Prerequisites

Before you get started, ensure that your system meets the following prerequisites:

- Ubuntu Operating System (Tested on Ubuntu 22.04)
- Installed Nvidia drivers (Refer to: [How to Install Nvidia Drivers on Ubuntu 22.04](https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-22-04))
- Docker Engine (Installation guide: [Docker Engine Installation](https://docs.docker.com/engine/install/ubuntu/))
- Nvidia Container Toolkit (Setup guide: [Nvidia Container Toolkit Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#setting-up-nvidia-container-toolkit))
- (Optional) Nvidia CUDA Toolkit (Installation guide: [Install Nvidia CUDA Toolkit on Ubuntu 22.04](https://installati.one/install-nvidia-cuda-toolkit-ubuntu-22-04/))

## Installation Steps for Development

Follow these steps to set up your environment for UR5e robot development:

1. **Install Prerequisites**: Ensure that your system meets the prerequisites listed above, including Nvidia drivers, Docker Engine, and Nvidia Container Toolkit.

2. **Configure Docker**: To run Docker commands without using `sudo`, perform the following steps:

    ```bash
    sudo groupadd docker
    sudo usermod -aG docker ${USER}
    reboot
    ```

3. **Clone the Repository**: Clone this repository to your local machine using the following command:

    ```bash
    git clone -b minimal https://github.com/tammerb/HOMESTRI-UR5e-Robotiq2f85.git
    ```

4. **Install Visual Studio Code Extensions**: Install the Docker extension and Remote Development extension for Visual Studio Code.

5. **Run the Docker Container**: Navigate to the root folder of the cloned repository and run the following command to start the Docker container:

    ```bash
    docker-compose up
    ```

6. **Attach Visual Studio Code**: Open Visual Studio Code, go to the Docker tab, right-click on the container, and select "Attach VSCode." This action will provide you with a Visual Studio Code environment within the container, preconfigured with ROS settings.

## Using the Docker Container

The Docker container sets up a ROS workspace in `/root/catkin_ws`.

There are two UR5e robots used in the lab. "a_bot" refers to the UR5e with a Robotiq 2F-85 Gripper, and "b_bot" refers to the UR5e with the OnRobot RG2-FT Gripper.

Within this workspace, you'll find the custom configuration packages specific to running the robots in the lab:

- `catkin_ws/src/homestri_ur/homestri_description`: Contains the URDF of the UR5e robots and grippers in the lab. Macros are available for importing the robots into your own URDFs.
- `catkin_ws/src/homestri_ur/homestri_bringup`: Contains launch files for bringing up both "a_bot" and "b_bot," including ROS controllers config files and calibration files.
- `catkin_ws/src/homestri_ur/homestri_a_bot_moveit_config`: Generated `moveit_config` package for "a_bot."
- `catkin_ws/src/homestri_ur/homestri_b_bot_moveit_config`: Generated `moveit_config` package for "b_bot."

The workspace also includes extra ROS package dependencies as git submodules in `catkin_ws`:

- `catkin_ws/src/controllers/cartesian_controllers`: Provides Cartesian motion, force, and compliance controllers for the `ros_control` framework.
- `catkin_ws/src/controllers/low_pass_force_torque_sensor_controller`: ROS package for controlling force-torque sensors via `ros_control` with an integrated low-pass filter.
- `catkin_ws/src/gazebo_pkgs/gazebo_gripper_action_controller`: ROS control gripper action controller for Gazebo.
- `catkin_ws/src/gazebo_pkgs/roboticsgroup_upatras_gazebo_plugins`: Contains the MimicJointPlugin, a simple model plugin for Gazebo to add mimic joint functionality.
- `catkin_ws/src/onrobot`: Contains ROS packages for controlling the OnRobot RG2-FT gripper.
- `catkin_ws/src/robotiq`: Contains ROS packages for controlling the Robotiq 2F-85 Gripper.
- `catkin_ws/src/universal_robots/Universal_Robots_Client_Library`: A C++ library for accessing Universal Robots interfaces.
- `catkin_ws/src/universal_robots/Universal_Robots_ROS_Driver`: UR5e ROS driver.
- `catkin_ws/src/universal_robots/universal_robot`: ROS-Industrial Universal Robot meta-package. Contains UR5e URDF.

## How to Run Robots

Note that the same commands for "a_bot" also apply for "b_bot"—just swap the names.

- Run fake robot in MoveIt: `roslaunch homestri_a_bot_moveit_config demo.launch`

- Run simulated robot in Gazebo: `roslaunch homestri_a_bot_moveit_config demo_gazebo.launch`

### Running the Real Robot

Follow these steps to run the real robot:

1. **Set Up External Control URCap**: Follow the steps for setting up external control URCap and programming on the UR teach pendant. This only needs to be done once. Refer to [this guide](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap).

2. **Calibrate the Real Robot**: Calibrate the real robot using the following command, replacing `<a_bot robot ip>` with the actual IP of the "a_bot" robot: `roslaunch homestri_bringup a_bot_calibration.launch robot_ip:=<a_bot robot ip>`

3. **Start Up the Real Robot**: Launch the real robot using the following command, replacing `<a_bot robot ip>` with the actual IP of the "a_bot" robot and `<host computer ip>` with the IP of your host computer: `roslaunch homestri_bringup a_bot_bringup.launch robot_ip:=<a_bot robot ip> reverse_ip:=<host computer ip>`

4. **Run the External Control Program**: Run the external control program you have set up on the teach pendant.

### Running Different Controllers for the UR5e

There are various controllers available for the robot as part of ROS control. You can find their configurations in `catkin_ws/src/homestri_ur/homestri_bringup/config/a_bot_controllers.yaml` (or `b_bot_controllers.yaml`).

To work with different controllers:

1. Make sure the real robot is running with ROS.

2. Open up `rqt_gui` using the command: `rosrun rqt_gui rqt_gui`.

3. In `rqt_gui`, go to `Plugins` -> `Robot Tools` -> `Controller Manager`.

4. Select the namespace of the controller manager.

5. To switch to a different controller, select the desired controller from the drop-down menu or radio buttons provided in the plugin interface.

   **Note**: Before switching to a different controller, ensure that the current running controller, if it shares resources, is turned off to avoid conflicts.

6. In `rqt_gui`, go to `Plugins` -> `Configuration` -> `Dynamic Reconfigure`. Experiment with different configuration values of the controllers.

## How to Use This Workspace for Your Own Project

1. **Create Your Dockerfile**: Create a new Dockerfile for your project and use `thbarkouki/homestri-ur:{specific tag}` as the base image. You can find available tags in the [Docker Hub repository](https://hub.docker.com/r/thbarkouki/homestri-ur/tags). Customize your Dockerfile to install your own project-specific dependencies.

2. **Build Your Docker Image**: Build your Docker image using your newly created Dockerfile.

3. **Customize ROS Packages**: Customize or extend the ROS packages within `catkin_ws/src/homestri_ur` to fit your project's needs. Modify existing packages or add new ones as required.

4. **Create Docker Compose Configuration**: Create a `docker-compose.yml` file for your project, specifying the necessary services and configurations. This could include additional ROS nodes, launch files, and environment settings.

5. **Run Your Docker Container**: Use `docker-compose` to run your Docker container based on your custom configuration.