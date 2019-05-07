/*
 * decawave_node.cpp
 * ROS interface to Decawave class
 * VERSION: 1.0
 * Last changed: 2019-04-01
 * Authors: Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainers: Amalia Schwartzwald <schw1818@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Native_Libs
#include <string>

// Custom_Libs
#include "decawave/decawave.h"
#include <serial/serial.h>
#include "decawave/Range.h"

// Subscribers (inputs)
//    update_timer (Timer)
//      Update loop for reading / querying Decawave
//    sub_name1 (sub_name1_type): sub_name1_TOPIC_VALUE
//      sub_name1_desc

// Publishers (outputs)
//    gps_pub (nav_msgs/Odometry): "odometry/gps"
//      A simulated local position as if GPS UTM

// Parameters (settings)
//    frequency (double): default=50.0
//      The update frequency of the update loop
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::Publisher gps_pub;

// ROS Topics
std::string gps_topic = "decawave/Range";//"odometry/gps";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency = 50.0;

int port_num = 0; //TODO: MOVE TO LAUNCH/INPUT
// Global_Vars
Decawave piTag(port_num);
decawave_coordinate tagPos;
// Decawave piTag(1);
// decawave_coordinate tagPos;

int main(int argc, char** argv){
  // ROS_INFO("Test");
  // Init ROS
  ros::init(argc, argv, "decawave_node");
  nh = new ros::NodeHandle("~");

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  gps_pub = nh->advertise<decawave::Range>(gps_topic, 10);

  // Params
  nh->param<double>("frequency", frequency);
  //nh->param<param_name2_type>(param_name2_path, param_name2, param_name2_default;
  //nh->param<param_name3_type>(param_name3_path, param_name3, param_name3_default;

  // Initialize
  // get initial decwave data
  for(int i = 0; i < 10; ++i){
    // ROS_INFO("Updating Samples");
    piTag.updateSamples();
  }

  // Spin
  ros::spin();
}

std::string frame_id = std::to_string(port_num);
void update_callback(const ros::TimerEvent&){
  decawave::Range msg;

  // update decawave data
  piTag.updateSamples();

  // get tag position from the decawave
  tagPos = piTag.getPos();

  msg.distance = tagPos.x;
  msg.estimated_variance = 0.10;
  // fill out message header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
  msg.child_frame_id = "decawave2_link"; //change to correct part

  gps_pub.publish(msg);

  msg.distance = tagPos.y;
  msg.estimated_variance = 0.10;
  // fill out message header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
  msg.child_frame_id = "decawave3_link"; //change to correct part

  gps_pub.publish(msg);
}
