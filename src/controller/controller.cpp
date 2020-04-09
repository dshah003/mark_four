#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>



class MarkFourController
{
  public:
  MarkFourController();
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void TeleOpM4();

  private:
  ros::NodeHandle nh_;
  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

MarkFourController::MarkFourController():
  linear_(1),
  angular_(1)
  {
    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mark_four/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &MarkFourController::JoyCallback, this);
  }

  void MarkFourController::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
    vel_pub_.publish(twist);
  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  MarkFourController markfour_controller;
  ros::spin();
}

