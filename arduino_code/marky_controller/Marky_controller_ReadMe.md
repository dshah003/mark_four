This code is uploaded on to the arduino Mega2560 controller onboard the robot.  

### Functions:  
- **[MobilityCallback]** Subscribes to '/mark_four/cmd_vel' reads the velocity commands and converts them to pwn signal and further controls left and right servo motors.
- publishes pwm value to '/lin_vel'.

### Troubleshooting notes:  
- PWM of ~ 81 makes the motor stop.
