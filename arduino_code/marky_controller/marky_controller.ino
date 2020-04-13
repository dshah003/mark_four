
/*
 * Program that runs on the arduino. It subscribes to ROS nodes as required and performs mobility and Arm movements accordingly.
 */

#include <ros.h>
#include <std_msgs/Int8.h>
#include "geometry_msgs/Twist.h"
#include <Servo.h>

ros::NodeHandle nh;

const int left_wheel_pin = 2;
const int right_wheel_pin = 3;
const int time_of_velocity = 100;

const double a = 0.0051; // Derived these constants from practical experimentation data
const double b = 0.416;
const float L = 0.13;
std_msgs::Int8 linear_vel;

Servo LeftWheel;
Servo RightWheel;

ros::Publisher vel_pub("lin_vel", &linear_vel);


void MobilityCallback(const geometry_msgs::Twist& vel_msg) {
  nh.loginfo("Received velocity");
  double lin_vel = vel_msg.linear.x;
  double ang_vel = vel_msg.angular.z;
  int lin_pwm = 160;
  // Tested:
  // if(lin_vel < 0.2){
  //   lin_pwm = (lin_vel + b) / a;
  // }
  // linear_vel.data = lin_pwm;
  // vel_pub.publish(&linear_vel);
  // LeftWheel.write(lin_pwm);
  // RightWheel.write(166-lin_pwm);
  // delay(time_of_velocity);

  const int gain = 395;
  float left_wheel_value = gain*(lin_vel + (ang_vel*L)) + 81.5;
  float right_wheel_value = gain*(lin_vel - (ang_vel*L)) + 81.5;
  linear_vel.data = int(left_wheel_value);
  vel_pub.publish(&linear_vel);
  LeftWheel.write(left_wheel_value);
  RightWheel.write(166- right_wheel_value);
  // delay(time_of_velocity);
  return;  
}

ros::Subscriber<geometry_msgs::Twist> velocity_sub("/mark_four/cmd_vel", MobilityCallback);

void setup() {
  LeftWheel.attach(left_wheel_pin);
  RightWheel.attach(right_wheel_pin);
  
  LeftWheel.write(83);
  RightWheel.write(83);
  
  nh.initNode();

  nh.subscribe(velocity_sub);
  nh.advertise(vel_pub);
  
  nh.loginfo("Arduino Ready");
}

void loop() {  
    // LeftWheel.write(83);
    // RightWheel.write(83);
    nh.spinOnce();
    delay(1);    
}
