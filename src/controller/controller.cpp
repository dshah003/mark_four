#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

const double PI = 3.14159265359;

class MarkFourController
{
  public:
    MarkFourController();
    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void move(double speed, double distance, bool isForward);
    void rotate (double angular_speed, double angle, bool clockwise);
    double degrees2radians(double angle_in_degrees);
    // void setDesiredOrientation (double desired_angle_radians);
    // void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
    // void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);

  private:
    ros::NodeHandle nh_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};


MarkFourController::MarkFourController() {
  linear_ = 1;
  angular_ = 3;
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

/**
 * @brief      Provides vel commands to move in straight line
 *
 * @param[in]  speed      The speed
 * @param[in]  distance   The distance
 * @param[in]  isForward  Indicates if forward
 */
void MarkFourController::move(double speed, double distance, bool isForward) {
  geometry_msgs::Twist vel_msg;
  //set a random linear velocity in the x-axis
  if (isForward)
    vel_msg.linear.x =-abs(speed);
  else
    vel_msg.linear.x =abs(speed);
  vel_msg.linear.y =0;
  vel_msg.linear.z =0;

  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;

  double t0 = ros::Time::now().toSec();
  double current_distance = 0.0;
  
  ros::Rate loop_rate(50);
  do{
    vel_pub_.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_distance = speed * (t1-t0);
    ros::spinOnce();
    loop_rate.sleep();
    //std::cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
  } while(current_distance<distance);
  vel_msg.linear.x = 0;
  vel_pub_.publish(vel_msg);
  ROS_INFO("Move Executed");
  return;
}

/**
 * @brief      Provides vel commands to rotate the robot.
 *
 * @param[in]  angular_speed   The angular speed
 * @param[in]  relative_angle  The relative angle
 * @param[in]  clockwise       The clockwise
 */
void MarkFourController::rotate (double angular_speed, double relative_angle, bool clockwise) {
  geometry_msgs::Twist vel_msg;
  //set a random linear velocity in the x-axis
  vel_msg.linear.x =0;
  vel_msg.linear.y =0;
  vel_msg.linear.z =0;
  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if (clockwise)
    vel_msg.angular.z = -abs(angular_speed);
  else
    vel_msg.angular.z = abs(angular_speed);

  double current_angle = 0.0;

  double t0 = ros::Time::now().toSec();
  ros::Rate loop_rate(10);
  do{
    vel_pub_.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_angle = angular_speed * (t1-t0);
    ros::spinOnce();
    loop_rate.sleep();
  }while(current_angle<relative_angle);

  vel_msg.angular.z =0;
  vel_pub_.publish(vel_msg);
  ROS_INFO("Done Rotating");
  return;
}

double degrees2radians(double angle_in_degrees) {
  return angle_in_degrees *PI /180.0;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  MarkFourController markfour_controller;

  // ROS_INFO("\n \n *** TESTING *** \n \n ");
  // double speed, angular_speed;
  // double distance, angle;
  // bool isForward, clockwise;
  // while(true) {
    // std::cout<<"Enter speed: ";
    // std::cin>>speed;
    // std::cout<<"Enter distance: ";
    // std::cin>>distance;
    // std::cout<<"forward?: ";
    // std::cin>>isForward;
    // markfour_controller.move(speed, distance, isForward);
    // ros::Duration(3).sleep();
    // ROS_INFO("\n\n*MOVED FORWARD*\n\n");

    // std::cout<<"enter angular velocity (degree/sec): ";
    // std::cin>>angular_speed;
    // std::cout<<"enter desired angle (degrees): ";
    // std::cin>>angle;
    // std::cout<<"clockwise ?: ";
    // std::cin>>clockwise;
    // markfour_controller.rotate(degrees2radians(angular_speed), 
    //     degrees2radians(angle), clockwise);
    // ros::Duration(3).sleep();
    // ROS_INFO("\n\n*DONE TURNING*\n\n");
      
  ros::spin();
}

