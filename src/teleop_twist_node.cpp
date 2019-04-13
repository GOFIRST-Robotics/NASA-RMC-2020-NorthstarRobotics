/*
 * teleop_twist_node.cpp
 * INSERT_DESC_HERE
 * VERSION: VERSION_NO
 * Last changed: 2018-09-24
 * Authors: My Name <myname@umn.edu>
 * Maintainers: Goldy <goldy@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Joy>
#include <geometry_msgs/Twist>

// Native_Libs
#include <string>

// Subscribers (inputs)
//    sub_name1 (sub_name1_type): sub_name1_TOPIC_VALUE
//      sub_name1_desc
//    joy_sub (sensor_msgs/Joy): joy
//      Joystick messages to be translated to velocity commands.
// Publishers (outputs)
//    pub_name1 (pub_name1_type): pub_name1_TOPIC_VALUE
//      pub_name1_desc
//    cmd_vel_pub (geometry_msgs/Twist): cmd_vel
// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::Publisher cmd_vel_pub;


// ROS Topics
std::string joy_topic = "joy";
std::string cmd_vel_topic = "cmd_vel";

// ROS Callbacks
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

// ROS Params
// // For normal types: string, int, float, bool
//param_name1_type param_name1 = param_name1_default;
//int enable_button = 0 ;

// Global_vars
int linear_axis = 1;
int angular_axis = 2;
double linear_scale = 1.0;
double angular_scale = 1.0;
//std::map<std::string, int> axis_linear_map;
//std::map<std::string, int> axis_angular_map;
//std::map<std::string, double> scale_linear_map;
//std::map<std::string, double> scale_angular_map;

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, FILE_NAME);
  nh = new ros::NodeHandle("~");
  
  // Subscribers
  //ros::Subscriber sub_name1_sub = nh->subscribe(sub_name1_topic, sub_name1_BUFLEN, sub_name1_callback);
  ros::subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);
  
  // Publishers
  //pub_name1_pub = nh->advertise<pub_name1_typeLHS::pub_name1_typeRHS>(pub_name1_topic, pub_name1_BUFLEN);
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  
  // Params
  //nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  //nh->param<int>( , linear_axis, 1);
  //nh->param<int>( , angular_axis, 2);
  
  // Spin
  ros::spin();
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  geometry_msgs::Twist cmd_vel_msg;
  
  cmd_vel_msg.linear.x = msg.axes[1] // This gets changed, forward vector
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = msg.axes[2]; // This spin
  cmd_vel_pub.publish(cmd_vel_msg);
}
/*
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
//:BEGIN sub_name2_callback

//:END
}
