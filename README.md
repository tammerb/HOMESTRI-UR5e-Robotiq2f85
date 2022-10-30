# HOMESTRI-UR5e-Robotiq2f85

Docker file and docker-compose for ROS noetic full, plus supporting packages for the UR5e and Robotiq 2F-85 gripper. The compose file starts two containers: the one running ROS and another running [noVNC](https://novnc.com/info.html) to use the GUI-based tools rviz and Gazebo.

- Install either [docker engine](https://docs.docker.com/engine/install/ubuntu/) (Linux) or [Docker Desktop](https://www.docker.com/).
- Clone this repo and navigate to the parent directory.
- Run `docker-compose up`.
- Go to [http://localhost:8080/vnc_auto.html](http://localhost:8080/vnc_auto.html) in a browser.
