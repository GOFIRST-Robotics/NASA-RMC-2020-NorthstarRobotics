/*
 * teleop_twist_node.cpp
 * This is a node that is supposed to get the joystick values from a joystick topic and output them to /cmd_vel
 * VERSION: VERSION_NO
 * Last changed: 2019_11_16
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

/**
 *  Subscribers (inputs)
 *    joy_sub (sensor_msgs/Joy): joy
 *      Joystick messages to be translated to velocity commands.
 * Publishers (outputs)
 *    cmd_vel_pub (geometry_msgs/Twist): cmd_vel
 * Parameters (settings)
 *    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)
 *    linear_scale (double): default = 1.0
 *      Meters/sec at full tilt
 *    angular_scale (double): default = 1.0
 *      Rad/sec at full tilt
 *    twist_topic (string): default = /cmd_vel
 *      Output topic for Twist messages
 */



// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher cmd_vel_pub;

// ROS Topics
std::string joy_topic = "joy";

// ROS Callbacks
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

// ROS Params
double linear_scale = 1.0;
double angular_scale = 1.0;
std::string twist_topic = "/cmd_vel";

// Global_vars
const int linear_axis = 1;
const int angular_axis = 3;

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "teleop_twist_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Subscribers
  ros::Subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);
  
  // Publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(twist_topic, 1);
  
  // Params
  //nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  pnh->param<double>("linear_scale", linear_scale, linear_scale);
  pnh->param<double>("angular_scale", angular_scale, angular_scale); 
  pnh->param<std::string>("twist_topic", twist_topic, twist_topic); 

  // Spin
  ros::spin();
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  geometry_msgs::Twist cmd_vel_msg;
  
  cmd_vel_msg.linear.x = (msg->axes[linear_axis])*linear_scale; // Forward speed
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = (msg->axes[angular_axis])*angular_scale; // This spin
  cmd_vel_pub.publish(cmd_vel_msg);
}