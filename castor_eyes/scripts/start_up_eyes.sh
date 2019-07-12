#! /bin/sh
source /opt/ros/kinetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
roscore
rosrun castor_eyes eyes.py
