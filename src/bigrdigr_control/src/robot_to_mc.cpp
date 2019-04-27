/*
 * robot_to_mc.cpp
 * attempts to put everything that moves data between mission control and the robot that is to be done on the robot's side (with the exception of video) in one node
 * VERSION: VERSION_NO
 * Last changed: 2019-04-24
 * Authors: Michael <lucke096@umn.edu>
 * Maintainers: Michael <lucke096@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>

// Native_Libs
#include <string>

// Custom Library
#include "telecom/telecom.h"
#include "formatter-string/Formatter.hpp"
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
telecomm::Telecomm mccomm;
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
  ros::init(argc, argv, "robot_to_mc");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  
  Formatter fmt = Formatter({cmd_msg_fmt, cmd_fmt});
  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  //ros::Subscriber sub_name2_sub = nh->subscribe(sub_name2_topic, sub_name2_BUFLEN, sub_name2_callback);

  // Publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::Twist(cmd_vel, 1);
  //pub_name2_pub = nh->advertise<pub_name2_typeLHS::pub_name2_typeRHS>(pub_name2_topic, pub_name2_BUFLEN);

  // Params
  pnh->param<double>("frequency", frequency);
  pnh->param<std::string>("dst_addr", dst_addr);
  pnh->param<int>("dst_port", dst_port);
  pnh->param<int>("src_port", src_port);
  //nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  mccomm = new telecomm::Telecomm(dst_addr, dst_port, src_port);
  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&){
  // joy_pub
  geometry_msgs::Twist cmd_vel_msg;
  mccomm.update() 
  if (mccomm.isCommClosed()){
    mccomm.reboot();
  }
  if (mccomm.recvAvail()){
    data=mccomm.recv();
  }
  std::vector<IV_float> vals = parseFloat(data, "cmd_vel_fmt", "cmd_fmt"); 
  cmd_vel_msg.linear.x = vals[0].v;
  cmd_vel_msg.linear.y = 0;
  cmd_vel_msg.linear.z = 0;
  cmd_vel_msg.angular.x = 0;
  cmd_vel_msg.angular.y = 0;
  cmd_vel_msg_angular.z = vals[1].v;
  cmd_vel_pub.publish(cmd_vel_msg);

  // other pub
}

/*
void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg){
//:BEGIN sub_name1_callback

//:END
}*/
