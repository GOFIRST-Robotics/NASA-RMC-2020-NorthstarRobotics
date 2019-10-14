
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
#include <sensor_msgs/Joy.h>

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
std::string joy_topic = "joy";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
double frequency = 50.0;
std::string dst_addr = "127.0.0.1";
int dst_port = 5017;
int src_port = 5018;

// Global_Vars
Telecom *digr_com;
Formatter *fmt;
std::string recv_msg;
std::vector<IV_float> cmd_vals = {{0,0}, {1,0}};
std::vector<IV> btn_vals = {{0,0}};
std::vector<IV> other_vals = {{0,0}};
// Formatters
val_fmt axis_msg_fmt = {
  "joy_msg",
  '!',
  4,
  0,
  2000,  //max_val
  1000,  //offset
  1000 //scale
};

val_fmt axis_fmt = {
  "joy_In",
  '@',
  6,
  -32767, // Minval
    32767, // Maxval
    0, // offset
    32767 // range
};
/*
val_fmt btn_fmt = {
  "btn_In",
  '#',
  1,
  0,
  255,
  0,
  255
}
*/
val_fmt btn_msg_fmt = {
  "btn_msg",
  '#',
  1,
  0,
  255,
  0,
  255 
};
val_fmt something_fmt = { //for the other set of axes
  "???_msg",
  '?',
  1,
  0,
  8,
  0,
  8
};
int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "mc_joy_to_robot");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Params
  pnh->param<std::string>("dst_addr", dst_addr);
  pnh->param<int>("dst_port", dst_port);
  pnh->param<int>("src_port", src_port);

  // Init variables
  fmt = new Formatter({axis_msg_fmt, axis_fmt, btn_msg_fmt, something_fmt});
  digr_com = new Telecom(dst_addr, dst_port, src_port);
  // Error checking here

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  ros::Subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);

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

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  std::cout << "calling joy_callback" << std::endl;
  if (digr_com->isComClosed()){
    digr_com->reboot();
  }
  btn_vals[0].v = 128*msg->buttons[7] + 64*msg->buttons[6] + 32*msg->buttons[5] + 16*msg->buttons[4] + 8*msg->buttons[3] + 4*msg->buttons[2] + 2*msg->buttons[1] + msg->buttons[0];  
  //std::cout << "this is doing a callback with linear/angular" << msg->linear.x << msg->angular.z;
  cmd_vals[0].v = msg->axes[1];
  cmd_vals[1].v = msg->axes[2];
  
  other_vals[0].v = msg->axes[4]+1+3*(msg->axes[5]+1);
  fmt->addFloat("joy_msg", cmd_vals, "joy_In");
  fmt->add("btn_msg", btn_vals, "btn_msg");
  fmt->add("???_msg", other_vals, "???_msg");
  digr_com->send(fmt->emit());
}

/*
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
}
*/
