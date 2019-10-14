
/*
 * robot_joy_to_mc_node.cpp
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
#include <sensor_msgs/Joy.h>

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
//    joy_pub (sensor_msgs/Joy): joy
//      publishes data from joysticks obtained over the internet

// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher joy_pub;

// ROS Topics
std::string joy_topic = "joyout";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
//void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
//void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);

// ROS Params
double frequency = 100.0;
std::string dst_addr = "127.0.0.1";
int dst_port = 5006;
int src_port = 5005;

// Global_Vars
Telecom *mc_com;
Formatter *fmt;
std::string recv_msg;

// Formatters

val_fmt joy_msg_fmt = {
        "joy_msg",
        '!',
        4,
        0,
        2000,  //max_val
        1000,  //offset
        1000 //scale
};
val_fmt joy_fmt = {
        "Joy_In",
        '@',
        6,
        -32767, // Minval
        32767, // Maxval
        0, // offset
        32767 // range
};
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
  fmt = new Formatter({joy_msg_fmt, joy_fmt, btn_msg_fmt, something_fmt});
  mc_com = new Telecom(dst_addr, dst_port, src_port);
  
  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  //ros::Subscriber sub_name2_sub = nh->subscribe(sub_name2_topic, sub_name2_BUFLEN, sub_name2_callback);

  // Publishers
  joy_pub = nh->advertise<sensor_msgs::Joy>(joy_topic, 1);
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
  sensor_msgs::Joy joy_msg;
  std::vector<IV_float> vals = fmt->parseFloat(recv_msg, "joy_msg_fmt", "joy_fmt"); 
  joy_msg.axes[1] = vals[0].v;
  joy_msg.axes[2] = vals[1].v;
  joy_msg.axes[3] = 0;
  joy_msg.axes[0] = 0;
  std::vector<IV> morevals = fmt->parse(recv_msg, "???_msg", "???_msg");
  joy_msg.axes[4] = morevals[0].v%3-1;
  joy_msg.axes[5] = morevals[0].v/3-1;
  std::vector<IV> btnval = fmt->parse(recv_msg, "btn_msg", "btn_msg");
  int btnvals = btnval[0].v;
  int buttons[12] = {btnvals % 2, (btnvals/2) % 2, (btnvals/4) % 2, (btnvals/8) % 2, (btnvals/16) % 2, (btnvals/32) % 2, (btnvals/64) % 2, (btnvals/128) % 2, 0, 0, 0, 0};
  for(int i=0; i<12;i++){
    joy_msg.buttons[i] = buttons[i];
  }
  joy_pub.publish(joy_msg);

  // other pub
}

/*
void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg){
//:BEGIN sub_name1_callback

//:END
}*/
