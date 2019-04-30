/*
 * robot_to_mc_node.cpp
 * Uses telecom to TX/RX ROS data from robot to the Mission Control (MC)
 * Does not transmit video
 * VERSION: 0.0.0
 * Last changed: 2019-04-28
 * Authors: Michael Lucke <lucke096@umn.edu>
 * Maintainers: Michael Lucke <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Native_Libs
#include <string>
#include <vector>

// Custom Library
#include "telecom/telecom.h"
#include "formatter_string/formatter.hpp"

// Subscribers (inputs)
//    some_sensor_sub (sub_name2_type): sub_name2_TOPIC_VALUE
//      subscribes to some sensor topics with data we want to transmit if we find one

// Publishers (outputs)
//    cmd_vel_pub (geometry_msgs/Twist): cmd_vel
//      publishes data from joysticks obtained over the internet

// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher cmd_vel_pub;

// ROS Topics
std::string joy_topic = "cmd_vel";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
double frequency = 100.0;
std::string dst_addr;
int dst_port;
int src_port;

// Global_Vars
Telecom *mc_com;
Formatter *fmt;
std::string recv_msg;

// Formatters
val_fmt cmd_msg_fmt = {
  "cmd_vel_msg",
  '!',
  4,
  0,
  2000,  //max_val
  1000,  //offset
  1000 //scale
};
val_fmt cmd_fmt = {
  "cmd_In",
  '@',
  6,
  -32767, // Minval
    32767, // Maxval
    0, // offset
    32767 // range
};

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "robot_to_mc_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Params
  pnh->param<double>("frequency", frequency);
  pnh->param<std::string>("dst_addr", dst_addr);
  pnh->param<int>("dst_port", dst_port);
  pnh->param<int>("src_port", src_port);
  //nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  
  // Init variables
  fmt = new Formatter({cmd_msg_fmt, cmd_fmt});
  mc_com = new Telecom(dst_addr, dst_port, src_port);
  
  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  //ros::Subscriber sub_name2_sub = nh->subscribe(sub_name2_topic, sub_name2_BUFLEN, sub_name2_callback);

  // Publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>(joy_topic, 1);
  //pub_name2_pub = nh->advertise<pub_name2_typeLHS::pub_name2_typeRHS>(pub_name2_topic, pub_name2_BUFLEN);

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&){
  // Read from mc_com for msg
  mc_com->update();
  if (mc_com->isComClosed()){
    mc_com->reboot();
  }
  if (mc_com->recvAvail()){
    recv_msg = mc_com->recv();
  }

  // joy_pub
  geometry_msgs::Twist cmd_vel_msg;
  std::vector<IV_float> vals = fmt->parseFloat(recv_msg, "cmd_vel_fmt", "cmd_fmt"); 
  cmd_vel_msg.linear.x = vals[0].v;
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg.angular.z = vals[1].v;
  cmd_vel_pub.publish(cmd_vel_msg);

  // other pub
}

/*
void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg){
//:BEGIN sub_name1_callback

//:END
}*/
