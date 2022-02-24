#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://10.1.100.88:11311
export ROS_IP=10.1.100.92
roslaunch sensor_launchers start_sensor.launch