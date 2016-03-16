# head_darwin_tb
This repository contain ROS files for head joints of darwin-op robot

##Installation

- Install ROS
- Install Dynamixel packages

```
sudo apt-get install ros-DISTRO-dynamixel*
```
(Replace "DISTRO" with the ROS version you want to use, in our case "indigo")

* Install pyserial
```
sudo apt-get install python-pip
sudo pip install pyserial 
```
* Remove deprecated functions from dynamixel_io

```
cd /opt/ros/DISTRO/lib/python2.7/dist-packages/dynamixel_driver
sudo chmod u=rw dynamixel_io.py
sudo vim dynamixel_io.py

After opening VIM press ESC, type "/self.ser" and press ENTER, 
you will go to the following code which you should comment with "#":

self.ser = serial.Serial(port)
self.ser.setTimeout(0.015)
self.ser.baudrate = baudrate

After that code you should add this line:

self.ser = serial.Serial(port, baudrate, timeout=0.015)
```

## Usage
```
cd ~/catkin_ws/src
git clone https://github.com/TauraBots/head_darwin_tb.git
source ../devel/setup.bash
cd head_darwin_tb/launch
#first start de dynamixel controller
roslaunch head_darwin_tb dx_controller.launch
#start the head controller
roslaunch head_darwin_tb start_controller.launch
#run the head test
cd ../src/
python trajectory_client.py
#and go crazy
```

## Some tips
```
#Move the head about 28 degrees
rostopic pub -1 /head_pan_controller/command std_msgs/Float64 -- -0.5
#Disable the torque of the pan motor
rosservice call /head_pan_controller/torque_enable False

```



