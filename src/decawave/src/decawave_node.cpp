
/*
 * decawave_node
 *
 * VERSION_#: 1.0
 * Last changed: Tue Jan 15 2019
 * AUTHORS: Amalia Schwartzwald schw1818@umn.edu
 * MAINTAINERS: Amalia Schwartzwald schw1818@umn.edu
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

// Custom_Libs
#include "decawave/decawave.h"
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <serial/serial.h>

// Subscribers (inputs):

// Publishers (outputs):
//  odometry (nav_msgs/Odometry): odom
//       Odometry message from decawave data

// Parameters (settings):

// ROS Node and Publishers
//ros::Publisher odom_pub;

int main(int argc, char *argv[]){

  // Init ROS
  ros::init(argc, argv, "dwn");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Time current_time;

  // Publishers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

  // Init Decawave
  Decawave piTag;
	coordinate tagPos;
  // get initial decwave data
  for(int i = 0; i < 10; ++i){
    piTag.updateSamples();
  }


  while (ros::ok())
  {
    // new message
    nav_msgs::Odometry msg;

    // update decawave data
    piTag.updateSamples();

    // get tag position from the decawave
    tagPos= piTag.getPos();

    // put the position data into the message
    msg.pose.pose.position.x = tagPos.x;
    msg.pose.pose.position.y = tagPos.y;

    // fill out message header
    current_time = ros::Time::now();
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link"; //change to correct part

    // publish the message with the odom_pub publisher
    odom_pub.publish(msg);

    // Spin and Sleep
    ros::spinOnce();
    loop_rate.sleep();
  }
}


