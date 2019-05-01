/*
 * joy_to_onboard_direct_node.cpp
 * Converts joystick to raw canbus msgs according to motor config.
 * VERSION: 0.0.0
 * Last changed: 2019_4_30
 * Authors: Jude Sauve <sauve031@umn.edu>
 * Maintainers: Jude Sauve <sauve031@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_msgs/Frame.h>

// Native_Libs
#include <string>
#include <stdint.h>
typedef int32_t S32;
typedef uint32_t U32;

// Subscribers (inputs)
//    joy_sub (sensor_msgs/Joy): joy
//      Joystick messages to be translated to canbus msgs
//
// Publishers (outputs)
//    can_pub (can_msgs/Frame): sent_messages
//
// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)
//      Insert desc


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher can_pub;

// ROS Topics
std::string joy_topic = "joy";
std::string can_topic = "sent_messages";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);

// ROS Params
double frequency = 2.0;
double linear_scale = 1.0;
double angular_scale = 1.0;
double lift_current = 4.0;
double trans_conv_current = 2.0;
double digger_current = 8.0;
double hold_conv_current = 3.0;

// Global_vars
void send_can(U32 id, S32 data);
bool buttons[12] = {0};
double axes[6] = {0.0};
double motors[9] = {0.0};
bool trans_conv = false;
bool digger = false;

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "joy_to_onboard_direct_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  // Params
  pnh->param<double>("frequency", frequency);
  pnh->param<double>("linear_scale", linear_scale);
  pnh->param<double>("angular_scale", angular_scale); 
  pnh->param<double>("lift_current", lift_current); 
  pnh->param<double>("trans_conv_current", trans_conv_current); 
  pnh->param<double>("digger_current", digger_current); 
  pnh->param<double>("hold_conv_current", hold_conv_current); 
  
  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  ros::Subscriber joy_sub = nh->subscribe(joy_topic, 1, joy_callback);
  
  // Publishers
  can_pub = nh->advertise<can_msgs::Frame>(can_topic, 100);

  // Spin
  ros::spin();
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
  // Process toggle
  if(buttons[0] != msg->buttons[0]){
    trans_conv = !trans_conv;
  }
  if(buttons[1] != msg->buttons[1]){
    digger = !digger;
  }
  // Process buttons
  for(int i = 0; i < 12; ++i){
    buttons[i] = msg->buttons[i];
  }
  // Process axes
  for(int i = 0; i < 6; ++i){
    axes[i] = msg->axes[i];
  }
}

void update_callback(const ros::TimerEvent&){
  // Process drive train
  send_can(0x101, (axes[1]*linear_scale - axes[2]*angular_scale)*32000);
  send_can(0x102, (axes[1]*linear_scale - axes[2]*angular_scale)*32000);
  send_can(0x103, (axes[1]*linear_scale + axes[2]*angular_scale)*32000);
  send_can(0x104, (axes[1]*linear_scale + axes[2]*angular_scale)*32000);
  // Process lift
  if(abs(buttons[5]) > 0.1){
    send_can(0x106, buttons[5] * lift_current * 1000.0);
    send_can(0x108, buttons[5] * lift_current * 1000.0);
  }else{
    send_can(0x106, 0);
    send_can(0x108, 0);
  }
  // Process trans conv
  send_can(0x107, trans_conv ? trans_conv_current * 1000.0 : 0.0);
  // Process digger
  send_can(0x109, digger ? digger_current * 1000.0 : 0.0);
  // Process hold conv
  if(buttons[6] > 0.1){
    send_can(0x105, hold_conv_current * 1000.0);
  }else if(buttons[7] > 0.1){
    send_can(0x105, hold_conv_current * -1000.0);
  }else{
    send_can(0x105, 0);
  }
}

void send_can(U32 id, S32 data){
  can_msgs::Frame can_msg;
  can_msg.is_rtr = false;
  can_msg.is_extended = true;
  can_msg.is_error = false;
  can_msg.dlc = 4U;
  can_msg.id = id;
  can_msg.data[3] = data & 0xFF;
  can_msg.data[2] = (data >> 8) & 0xFF;
  can_msg.data[1] = (data >> 16) & 0xFF;
  can_msg.data[0] = (data >> 24) & 0xFF;
  can_pub.publish(can_msg);
}
