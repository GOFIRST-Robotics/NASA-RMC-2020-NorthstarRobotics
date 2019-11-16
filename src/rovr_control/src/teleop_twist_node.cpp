/*
 * teleop_twist_node.cpp
 * This is a node that is supposed to get the joystick values from a joystick topic and output them to /cmd_vel
 * VERSION: VERSION_NO
 * Last changed: 2019_4_17
 * Authors: Michael <lucke096@umn.edu>
 * Maintainers: Michael <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// Native_Libs
#include <string>

// Subscribers (inputs)
//    joy_sub (sensor_msgs/Joy): joy
//      Joystick messages to be translated to velocity commands.
// Publishers (outputs)
//    cmd_vel_pub (geometry_msgs/Twist): cmd_vel
// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)
//    linear_scale (double): default = 1.0
//      Insert desc


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher cmd_vel_pub;

// ROS Topics
std::string joy_topic = "joy";
std::string cmd_vel_topic = "/rovr/cmd_vel";

// ROS Callbacks
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

// ROS Params
// // For normal types: string, int, float, bool
//param_name1_type param_name1 = param_name1_default;
//int enable_button = 0 ;
double linear_scale = 1.0;
double angular_scale = 1.0;

// Global_vars
const int linear_axis = 1;
const int angular_axis = 4;
/*const int decrease_angular_scale_button = 6;
const int decrease_linear_scale_button = 7;
const int increase_linear_scale_button = 5;
const int increase_angular_scale_button = 4;*/ //a bunch of debugging stuff I think
//double linear_scale = 1.0;
//double angular_scale = 1.0;
//std::map<std::string, int> axis_linear_map;
//std::map<std::string, int> axis_angular_map;
//std::map<std::string, double> scale_linear_map;
//std::map<std::string, double> scale_angular_map;

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "teleop_twist_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Subscribers
  ros::Subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);
  
  // Publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  
  // Params
  //nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  pnh->param<double>("linear_scale", linear_scale, angular_scale);
  pnh->param<double>("angular_scale", angular_scale, angular_scale); 
  //std::cout << "Yo, scales: " << linear_scale << std::endl;
  // Spin
  ros::spin();
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  geometry_msgs::Twist cmd_vel_msg;
  
  cmd_vel_msg.linear.x = (msg->axes[1])*linear_scale; // This gets changed, forward vector
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = (msg->axes[3])*angular_scale; // This spin
  cmd_vel_pub.publish(cmd_vel_msg);
  /*if (msg->buttons[decrease_linear_scale_button] == 1){
    linear_scale = linear_scale / 1.025;
  }
  if (msg->buttons[decrease_angular_scale_button] == 1){
    angular_scale = angular_scale / 1.025;
  }
  if (msg->buttons[increase_linear_scale_button] == 1){
    linear_scale = linear_scale * 1.025;
  }
  if (msg->buttons[increase_angular_scale_button] == 1){
    angular_scale = angular_scale * 1.025;
  }*/
}
/*
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
//:BEGIN sub_name2_callback

//:END
}
*/
