# Mark IV
(Readme constantly updated as the project progresses.)


An experimental robotic platform with mobility and robotic arm.  

### Functionality:  
- Implemented on ROS framework.
- Differential Drive actuated by 2 continuous rotation servo motors.  
- Manual Control via Joystick.


### Package requirments:  

**Joystick driver**

```sh
sudo apt-get install ros-melodic-joy
```

**kinect related packages**  

Install OpenNI and dependency

```sh
sudo apt-get install ros-melodic-openni-*  
sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev  

```

Install RTAB mapping package  

```sh
sudo apt-get install ros-melodic-rtabmap-ros   
```

To Test installation, connect the kinect sensor and run the following commands:  
```sh
roslaunch openni_launch openni.launch depth_registration:=true  
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"  

```  

**Navigation Stack**

Go to your catkin worspace/src folder and clone/install the following packages
```
git clone https://github.com/ros-planning/navigation.git
sudo apt-get install ros-melodic-depthimage-to-laserscan

```  
Do not forget to ```catkin_make``` once the packages are cloned.  

**Misc Supporting packages**

```sh
sudo apt-get install ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
sudo apt-get install urdf  
sudo apt-get install ros-melodic-tf2-sensor-msgs
sudo apt-get install libsdl-dev libsdl-image1.2-dev

```

Reference:  
[https://github.com/introlab/rtabmap/wiki/Installation]  
[http://wiki.ros.org/rtabmap_ros]



### Instructions

**To operate the robot,**  
Make sure arduino is connected to the computer via USB and running the script arduino_code/marky_controller.ino  
Make sure the joystick usb dongle is also connected.  
Use the following command to run the launch file.

```sh
roslaunch mark_four controller_teleop.launch
```

**To test the kinect**  

```sh
roslaunch openni_launch openni.launch  
rosrun rviz rviz   
```
For more info, refer: [https://wiki.ros.org/openni_launch]  


### Topics  

- /joy: Joystick commands are published by joy package and subscribed by controller node.
- /mark_four/cmd_vel: All the motion commands to the arduino node are published on this topic as velocity commands.

### Notes  

- No matter the source of input (joystick/autonomous), eventually all the motion related commands are convertered into velocity commands and published over /vel_cmd  
- Edit the "camera" argument in openni_launch/openni.launch from "camera" to "kinect"
