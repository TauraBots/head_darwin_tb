#!/bin/bash

cd ~/catkin_ws
source /opt/ros/indigo/setup.bash

#kill ros processes if already running
sudo killall roslaunch
sudo killall -9 rosmaster

#just to do the setup, sometimes we have problems with this
catkin_make
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB0
cd ~/catkin_ws/src

#launching the controller that connects the dynamixel motors
#with the opencm and the opencm with the computer
roslaunch head_darwin_tb/launch/dx_controller.launch &
sleep 5

#it actually start the motors that are defined on 
#head_darwin_tb/param/parm_head.yaml in our code!!!
roslaunch head_darwin_tb/launch/start_controller.launch &
sleep 2

#our test code!
python ~/catkin_ws/src/head_darwin_tb/src/trajectory_client.py

exit
