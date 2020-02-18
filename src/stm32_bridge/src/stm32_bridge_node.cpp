/*
 * stm32_bridge_node.cpp
 * Connects to the STM32 real-time controller
 * VERSION: 1.0.0
 * Last changed: 2020-02-04
 * Authors: Julia Schatz <schat127@umn.edu>
 * Maintainers: Julia Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2020 UMN Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>

// Native_Libs
#include <string>

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher can_pub;

// ROS Callbacks
void twist_callback(const geometry_msgs::Twist& msg);
void can_recv_callback(const can_msgs::Frame& msg);

// ROS Params
std::string pose_frame_id;
std::string twist_frame_id;
int covar_samples;

// Custom Types
typedef struct {
  float x;
  float y;
  float twist;
} OdomEntry;

// Global_Vars
std::vector<OdomEntry> odomHistory;

// Utility Methods
can_msgs::Frame make_frame(uint8_t sys_id, uint8_t cmd_id, uint8_t* data, uint8_t len);
int32_t buffer_pop_int32(uint8_t const* buffer, int* index);
void buffer_put_int32(uint8_t* buffer, int* index, int32_t value);

int main(int argc, char** argv) {
  // Init ROS
  ros::init(argc, argv, "stm32_bridge_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params

  // Subscribers
  ros::Subscriber can_sub = nh->subscribe("received_messages", 5, can_recv_callback);
  ros::Subscriber twist_sub = nh->subscribe("cmd_vel", 5, twist_callback);

  // Publishers
  can_pub = nh->advertise<can_msgs::Frame>("sent_messages", 5);

  // Spin
  ros::spin();
}

void twist_callback(const geometry_msgs::Twist& msg) {\
    double linear = msg.linear.x * 1000.0;
    double angular = msg.angular.z * 1000.0;
    int32_t v_linear = (int32_t) (linear); // mm/s
    int32_t v_angular = (int32_t) (angular); // mm/s
    uint8_t buffer[8];
    int index = 0;
    buffer_put_int32(buffer, &index, v_angular);
    buffer_put_int32(buffer, &index, v_linear);
    can_msgs::Frame my_frame = make_frame(100, 35, buffer, 8);

    my_frame.header.stamp = ros::Time::now();
    can_pub.publish(my_frame);
}

void can_recv_callback(const can_msgs::Frame& msg) {

}

can_msgs::Frame make_frame(uint8_t sys_id, uint8_t cmd_id, uint8_t* data, uint8_t len) {
  can_msgs::Frame frame;
  frame.id = (cmd_id << 8) | sys_id;
  frame.is_rtr = false; // We always send data frames
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = len;
  memcpy(&(frame.data), data, len);
  return frame;
}

int32_t buffer_pop_int32(uint8_t const* buffer, int* index) {
  int32_t buf;
  buf = buffer[(*index)++] << 24;
  buf |= buffer[(*index)++] << 16;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int32(uint8_t* buffer, int* index, int32_t const value) {
  buffer[(*index)++] = value >> 24;
  buffer[(*index)++] = value >> 16;
  buffer[(*index)++] = value >> 8;
  buffer[(*index)++] = value;
}