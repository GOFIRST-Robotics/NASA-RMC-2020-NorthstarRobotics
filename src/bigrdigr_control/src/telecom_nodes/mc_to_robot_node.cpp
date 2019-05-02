/*
 * mc_to_robot_node.cpp
 * Uses telecom to TX/RX ROS data from the Mission Control (MC) to the robot
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
//    joy_sub (sensor_msgs/Joy): joy
//      subscribes to joystick data
//      sub_name2_desc
//    sub_name3 (sub_name3_type): sub_name3_TOPIC_VALUE
//      sub_name3_desc

// Publishers (outputs)
//    pub_name1 (pub_name1_type): pub_name1_TOPIC_VALUE
//      pub_name1_desc
//    pub_name2 (pub_name2_type): pub_name2_TOPIC_VALUE
//      pub_name2_desc
//    pub_name3 (pub_name3_type): pub_name3_TOPIC_VALUE
//      pub_name3_desc

// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
//ros::Publisher pub_name3_pub;
// ROS Topics
std::string cmd_vel_topic = "cmd_vel";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
double frequency = 50.0;
std::string dst_addr = "127.0.0.1";
int dst_port = 5005;
int src_port = 5006;

// Global_Vars
Telecom *digr_com;
Formatter *fmt;
std::string recv_msg;
std::vector<IV_float> cmd_vals = {{0,0}, {1,0}};

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
  ros::init(argc, argv, "mc_to_robot_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Params
  pnh->param<std::string>("dst_addr", dst_addr);
  pnh->param<int>("dst_port", dst_port);
  pnh->param<int>("src_port", src_port);

  // Init variables
  fmt = new Formatter({cmd_msg_fmt, cmd_fmt});
  digr_com = new Telecom(dst_addr, dst_port, src_port);
  // Error checking here

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  ros::Subscriber cmd_vel_sub = nh->subscribe(cmd_vel_topic, 1, cmd_vel_callback);

  // Publishers

  // Spin
  ros::spin();
  std::cout << "mc node initialized" << std::endl;
}

void update_callback(const ros::TimerEvent&){
  // Read from digr_com for msg
  digr_com->update();
  if (digr_com->isComClosed()){
    digr_com->reboot();
  }
  if (digr_com->recvAvail()){
    recv_msg = digr_com->recv();
  }


}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
  std::cout << "calling cmd_vel_callback" << std::endl;
  if (digr_com->isComClosed()){
    digr_com->reboot();
  }
  //std::cout << "this is doing a callback with linear/angular" << msg->linear.x << msg->angular.z;
  cmd_vals[0].v = msg->linear.x;
  cmd_vals[1].v = msg->angular.z;
  fmt->addFloat("cmd_vel_msg", cmd_vals, "cmd_In");
  digr_com->send(fmt->emit());
}

/*
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
}
*/
