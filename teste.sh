#!/bin/bash

cd ~/catkin_ws
source /opt/ros/indigo/setup.bash
sudo killall -9 rosmaster
catkin_make
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB0
cd ~/catkin_ws/src

roslaunch head_darwin_tb/launch/dx_controller.launch &
sleep 5
xdotool key enter #sudo apt-get install xdotools

roslaunch head_darwin_tb/launch/start_controller.launch &
sleep5
xdotool key enter

python head_darwin_tb/src/trajectory_client.py
exit
