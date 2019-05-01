/*
 * navx_node.cpp
 * Runs the Kauai Labs NavX, using modified NavX library
 * VERSION: 0.0
 * Last changed: 2019-04-01
 * Authors: Jude Sauve <sauve031@umn.edu>
 * Maintainers: Jude Sauve <sauve031@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Native_Libs
#include <string>

// Custom_Libs
#include "ahrs/AHRS.h"

// Subscribers (inputs)
//    update_timer (Timer)
//      Update loop for reading / querying IMU

// Publishers (outputs)
//    imu_pub (sensor_msgs/Imu): imu/data
//      The published imu data

// Parameters (settings)
//    frequency (double): default=50.0
//      The update frequency of the update loop
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::Publisher imu_pub;

// ROS Topics
std::string imu_topic = "imu/data";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

// ROS Params
double frequency = 50.0;
//param_name2_type param_name2 = param_name2_default;
//param_name3_type param_name3 = param_name3_default;

// Global_Vars
AHRS com = AHRS("/dev/ttyACM0");
// double global1 = 0.0;
// int    global2 = 0;
// std::string global3 = "";
// void myCustomFunc1(int i);
// bool myCustomFunc2(float f);

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "navx_node");
  nh = new ros::NodeHandle("~");

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  imu_pub = nh->advertise<sensor_msgs::Imu>(imu_topic, 10);

  // Params
  nh->param<double>("frequency", frequency);
  //nh->param<param_name2_type>(param_name2_path, param_name2, param_name2_default;
  //nh->param<param_name3_type>(param_name3_path, param_name3, param_name3_default;

  // Spin
  ros::spin();
}

void update_callback(const ros::TimerEvent&){
  sensor_msgs::Imu msg;
  
  msg.orientation.x = com.GetQuaternionX();
  msg.orientation.y = com.GetQuaternionY();
  msg.orientation.z = com.GetQuaternionZ();
  msg.orientation.w = com.GetQuaternionW();
  msg.orientation_covariance[0] = -1;
  //msg.orientation_covariance[3] = ;
  //msg.orientation_covariance[6] = ;
  
  msg.angular_velocity.x = 0;//com.GetPitch() * ((2.0 * 3.14159) / 360.0);
  msg.angular_velocity.y = 0;//com.GetRoll() * ((2.0 * 3.14159) / 360.0);
  msg.angular_velocity.z = com.GetRate() * ((2.0 * 3.14159) / 360.0);
  msg.angular_velocity_covariance[0] = -1;
  //msg.angular_velocity_covariance[3] = ;
  //msg.angular_velocity_covariance[6] = ;
  
  msg.linear_acceleration.x = com.GetWorldLinearAccelX() * 9.81;
  msg.linear_acceleration.y = com.GetWorldLinearAccelY() * 9.81;
  msg.linear_acceleration.z = com.GetWorldLinearAccelZ() * 9.81;
  msg.linear_acceleration_covariance[0] = 0.8825985;
  msg.linear_acceleration_covariance[3] = 0.8825985;
  msg.linear_acceleration_covariance[6] = 1.569064;
  
  imu_pub.publish(msg);
}
