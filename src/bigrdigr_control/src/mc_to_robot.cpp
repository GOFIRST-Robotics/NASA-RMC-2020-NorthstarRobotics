/*
 * mc_to_robot.cpp // // Note, global FILE_NAME is "example_node"
 * This node sends data from mission control to the robot and recieves data from the robot
 * VERSION: VERSION_NO
 * Last changed: 2019-04-27
 * Authors: Michael <lucke096@umn.edu>
 * Maintainers: Goldy <goldy@umn.edu>
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
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
std::string dst_addr;
int dst_port;
int src_port;
// Global_Vars
telecomm::Telecomm digr_comm;

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

std::vector<IV_float> cmd_vals = {{0,0}, {1,0}};

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "mc_to_robot");
  nh = new ros::NodeHandle()
  pnh = new ros::NodeHandle("~");
  
  Formatter fmt = Formatter({cmd_msg_fmt, cmd_fmt});
  // Subscribers
  ros::Subscriber cmd_vel_sub = nh->subscribe(cmd_vel_topic, 1, cmd_vel_callback);

  // Publishers

  // Params
  pnh->param<std::string>("dst_addr", dst_addr);
  pnh->param<int>("dst_port", dst_port);
  pnh->param<int>("src_port", src_port);

  digr_comm = new telecomm::Telecomm(dst_addr, dst_port, src_port);
  // Spin
  ros::spin();
}


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
  digr_comm.update();
  cmd_vals[0].v = msg->linear.x;
  cmd_vals[1].v = msg->angular.z;
  fmt.addFloat("cmd_vel_msg", cmd_vals, "cmd_In");
  senddata = fmt.emit();
  if (digr_comm.isCommClosed()){
    digr_comm.reboot();
  }
  digr_comm::send(senddata);

}
/*
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
}
*/
