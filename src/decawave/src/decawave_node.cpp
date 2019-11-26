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
ros::NodeHandle * pnh;
ros::Publisher gps_pub;

// ROS Topics
std::string gps_topic = "decawave/Range";//"odometry/gps";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency = 50.0;
std::string port_name= "ttyACM0";
// for(int i = 0; i < argc; ++i){
// int port_num = atoi(argv[1]);
// }
// Global_Vars
Decawave *piTag;

int main(int argc, char** argv){
  // Params
  ros::param::get("~port_name",port_name);

  // Init this ROS node
  ros::init(argc, argv, port_name);
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");

  // Init Decawave
  piTag = new Decawave(port_name);

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  gps_topic = gps_topic + port_name;
  gps_pub = nh->advertise<decawave::Range>(gps_topic, 10);

  // Spin
  ros::spin();
}


void update_callback(const ros::TimerEvent&){
  // ROS_INFO("Publishing..");
  decawave::Range msg;

  // update decawave data
  std::vector<anchor> anchors = piTag->updateSamples();

  ros::param::get("~port_num",port_name);
  std::string frame_id = port_name;

  int j=anchors.size();
  int i=0;
  anchor m_anchor;
  while (i<j){
    m_anchor=anchors[i];//get new anchor
    msg.distance = ((float)m_anchor.distance)/1000;
    msg.distance_quality = m_anchor.distance_quality;
    msg.quality_factor=m_anchor.quality_factor;
    msg.position={((float)m_anchor.position[0])/1000,
      ((float)m_anchor.position[1])/1000,
      ((float)m_anchor.position[3])/1000};
    // fill out message header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "decawave" + frame_id;//"decawave_" + port_num;
    msg.child_frame_id = "decawave2_link"; //change to correct part
    gps_pub.publish(msg);
  }
}
