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

### Instructions

To operate the robot, use the following command to run the launch file.

```sh
roslaunch mark_four controller_teleop.launch
```

### Publishers


### Subscribers


### Notes  

- No matter the source of input (joystick/autonomous), eventually all the motion related commands are convertered into velocity commands and published over /vel_cmd