/*
   Program that runs on the arduino. It subscribes to ROS nodes as required and performs mobility and Arm movements accordingly.
*/

#include <ros.h>
#include <stdlib.h>
#include <std_msgs/Int32.h>
#include "geometry_msgs/Twist.h"
#include <Servo.h>

ros::NodeHandle nh;

const int left_wheel_pin = 2;
const int right_wheel_pin = 3;
const int time_of_velocity = 100;

const float m = 1963.93; // Derived these constants from practical experimentation data
const float c = 1400;
const float L = 0.13;  // distance between wheels.
const float R = 0.07;  // wheel radius.
const int stop_signal = 1400;
std_msgs::Int32 linear_vel;

Servo LeftWheel;
Servo RightWheel;

ros::Publisher vel_pub("lin_vel", &linear_vel);

int calPwm(float velocity) {
  int pwm = (int)(m * velocity + c);
  return pwm;
}

void MobilityCallback(const geometry_msgs::Twist& vel_msg) {
  nh.loginfo("Received velocity");
  double lin_vel = vel_msg.linear.x;
  double ang_vel = vel_msg.angular.z;
  int left_pwm, right_pwm;

  if (abs(lin_vel) > 0.17) { //restricting the max lin vel to 0.17
    lin_vel = 0.17;
  }
  if (abs(ang_vel) > 1.5) { //Restricting the max rotational vel to 1.5 rad/s
    ang_vel = 1.5;
  }

  float vel_ang = (ang_vel * R)/2;
  
    left_pwm = calPwm(lin_vel + vel_ang);
    right_pwm = calPwm(lin_vel - vel_ang);
    LeftWheel.writeMicroseconds(left_pwm);
    RightWheel.writeMicroseconds(2800 - right_pwm);
    char buff[10];
    nh.loginfo("Left Wheel: ");
    itoa(left_pwm, buff, 10);
    nh.loginfo(buff);
    nh.loginfo("Right Wheel: ");
    itoa((2800 - right_pwm), buff, 10);
    nh.loginfo(buff);
    
  linear_vel.data = int(left_pwm);
  vel_pub.publish(&linear_vel);
  return;
}

ros::Subscriber<geometry_msgs::Twist> velocity_sub("/mark_four/cmd_vel", MobilityCallback);

void setup() {
  LeftWheel.attach(left_wheel_pin);
  RightWheel.attach(right_wheel_pin);

  LeftWheel.writeMicroseconds(stop_signal);
  RightWheel.writeMicroseconds(stop_signal);

  nh.initNode();

  nh.subscribe(velocity_sub);
  nh.advertise(vel_pub);

  nh.loginfo("Arduino Ready");
}

void loop() {
//  LeftWheel.writeMicroseconds(stop_s/ignal);
//  RightWheel.writeMicroseconds(stop_sig/nal);
  nh.spinOnce();
  delay(2);
}
