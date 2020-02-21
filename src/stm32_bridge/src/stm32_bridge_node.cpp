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
#include <nav_msgs/Odometry.h>
#include <stm32_bridge/ACHOOCmd.h>
#include <stm32_bridge/GESUNDHEITExtendCmd.h>
#include <stm32_bridge/GESUNDHEITDoorCmd.h>
#include <stm32_bridge/GESUNDHEITSpeedCmd.h>
#include <stm32_bridge/SNEEZEDigSpeedCmd.h>
#include <stm32_bridge/SNEEZETransSpeedCmd.h>
#include <stm32_bridge/SNEEZEHomeCmd.h>
#include <stm32_bridge/BLESSYOUSpeedCmd.h>
#include <stm32_bridge/BLESSYOUPositionCmd.h>

// Native_Libs
#include <string>
#include <cmath>
#include <iostream>

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher can_pub;
ros::Publisher odom_pub;

// ROS Callbacks
void twist_callback(const geometry_msgs::Twist& msg);
void achoo_callback(const stm32_bridge::ACHOOCmd& msg);
void gesundheit_ext_callback(const stm32_bridge::GESUNDHEITExtendCmd& msg);
void gesundheit_door_callback(const stm32_bridge::GESUNDHEITDoorCmd& msg);
void gesundheit_speed_callback(const stm32_bridge::GESUNDHEITSpeedCmd& msg);
void sneeze_dig_speed_callback(const stm32_bridge::SNEEZEDigSpeedCmd& msg);
void sneeze_trans_speed_callback(const stm32_bridge::SNEEZETransSpeedCmd& msg);
void sneeze_home_callback(const stm32_bridge::SNEEZEHomeCmd& msg);
void blessyou_speed_callback(const stm32_bridge::BLESSYOUSpeedCmd& msg);
void blessyou_position_callback(const stm32_bridge::BLESSYOUPositionCmd& msg);
void can_recv_callback(const can_msgs::Frame& msg);

// ROS Params
int covar_samples;
std::string frame_id;
std::string child_frame_id;

// Custom Types
typedef struct {
  // Pose
  float x;
  float y;
  float theta;
  // Twist
  float dx;
  float omega;
} OdomEntry;

// Global_Vars
std::vector<OdomEntry> odomHistory;
int odom_seq = 0;
static const float DEG_TO_RAD = M_PI / 180.0F;

// Utility Methods
can_msgs::Frame make_frame(uint8_t sys_id, uint8_t cmd_id, uint8_t* data, uint8_t len);
int32_t buffer_pop_int32(uint8_t const* buffer, int* index);
void buffer_put_int32(uint8_t* buffer, int* index, int32_t value);
int16_t buffer_pop_int16(uint8_t const* buffer, int* index);
void buffer_put_int16(uint8_t* buffer, int* index, int16_t value);
void unpack_frame(const can_msgs::Frame& msg, uint8_t* sys_id, uint8_t* cmd_id, uint8_t* data, uint8_t* len);
void handle_drivetrain_msg(uint8_t cmd_id, uint8_t* data, uint8_t len);
bool calculate_covariance(boost::array<double, 36> &pose_mat, boost::array<double, 36> &twist_mat);

int main(int argc, char** argv) {
  // Init ROS
  ros::init(argc, argv, "stm32_bridge_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params
  pnh->param<std::string>("pose_frame_id", frame_id, "odom");
  pnh->param<std::string>("twist_frame_id", child_frame_id, "base_link");
  pnh->param<int>("covar_samples", covar_samples, 10);

  // Setup node
  odomHistory.resize(covar_samples);

  // Subscribers
  ros::Subscriber can_sub = nh->subscribe("received_messages", 5, can_recv_callback);
  ros::Subscriber twist_sub = nh->subscribe("cmd_vel", 5, twist_callback);
  ros::Subscriber achoo_ext_sub = nh->subscribe("achoo/extend", 5, achoo_callback);
  ros::Subscriber gesundheit_ext_sub = nh->subscribe("gesundheit/extend", 5, gesundheit_ext_callback);
  ros::Subscriber gesundheit_door_sub = nh->subscribe("gesundheit/door", 5, gesundheit_door_callback);
  ros::Subscriber gesundheit_speed_sub = nh->subscribe("gesundheit/speed", 5, gesundheit_speed_callback);
  ros::Subscriber sneeze_dig_speed_sub = nh->subscribe("sneeze/dig_speed", 5, sneeze_dig_speed_callback);
  ros::Subscriber sneeze_trans_speed_sub = nh->subscribe("sneeze/trans_speed", 5, sneeze_trans_speed_callback);
  ros::Subscriber sneeze_home_sub = nh->subscribe("sneeze/home", 5, sneeze_home_callback);
  ros::Subscriber blessyou_speed_sub = nh->subscribe("blessyou/speed", 5, blessyou_speed_callback);
  ros::Subscriber blessyou_position_sub = nh->subscribe("blessyou/position", 5, blessyou_position_callback);

  // Publishers
  can_pub = nh->advertise<can_msgs::Frame>("sent_messages", 5);
  odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 5);

  // Spin
  ros::spin();
}

void twist_callback(const geometry_msgs::Twist& msg) {
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
  std::cout << "Sending cmd vel msg" << std::endl;
  can_pub.publish(my_frame);
}

void achoo_callback(const stm32_bridge::ACHOOCmd& msg) {
  uint8_t buf[] = {msg.command};
  can_msgs::Frame my_frame = make_frame(101, 40, buf, 1);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void gesundheit_ext_callback(const stm32_bridge::GESUNDHEITExtendCmd& msg) {
  uint8_t buf[] = {msg.command};
  can_msgs::Frame my_frame = make_frame(102, 50, buf, 1);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void gesundheit_door_callback(const stm32_bridge::GESUNDHEITDoorCmd& msg) {
  uint8_t buf[] = {msg.command};
  can_msgs::Frame my_frame = make_frame(102, 52, buf, 1);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void gesundheit_speed_callback(const stm32_bridge::GESUNDHEITSpeedCmd& msg) {
  uint8_t buf[4];
  int idx = 0;
  buffer_put_int32(buf, &idx, msg.speed);
  can_msgs::Frame my_frame = make_frame(102, 51, buf, 4);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void sneeze_dig_speed_callback(const stm32_bridge::SNEEZEDigSpeedCmd& msg) {
  uint8_t buf[4];
  int idx = 0;
  buffer_put_int32(buf, &idx, msg.speed);
  can_msgs::Frame my_frame = make_frame(103, 61, buf, 4);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void sneeze_trans_speed_callback(const stm32_bridge::SNEEZETransSpeedCmd& msg) {
  uint8_t buf[4];
  int idx = 0;
  buffer_put_int32(buf, &idx, msg.speed);
  can_msgs::Frame my_frame = make_frame(103, 63, buf, 4);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void sneeze_home_callback(const stm32_bridge::SNEEZEHomeCmd& msg) {
  can_msgs::Frame my_frame = make_frame(103, 62, nullptr, 0);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void blessyou_speed_callback(const stm32_bridge::BLESSYOUSpeedCmd& msg) {
  uint8_t buf[] = {msg.speed};
  can_msgs::Frame my_frame = make_frame(104, 71, buf, 1);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}
void blessyou_position_callback(const stm32_bridge::BLESSYOUPositionCmd& msg) {
  uint8_t buf[] = {msg.position};
  can_msgs::Frame my_frame = make_frame(104, 72, buf, 1);

  my_frame.header.stamp = ros::Time::now();
  can_pub.publish(my_frame);
}

void can_recv_callback(const can_msgs::Frame& msg) {
  uint8_t sys_id;
  uint8_t cmd_id;
  uint8_t data[8];
  uint8_t len;
  unpack_frame(msg, &sys_id, &cmd_id, data, &len);
  switch (sys_id) {
    case 100: handle_drivetrain_msg(cmd_id, data, len); break;

  }
}

void handle_drivetrain_msg(uint8_t cmd_id, uint8_t* data, uint8_t len) {
  int idx = 0;
  if (cmd_id == 36 && len >= 6) { // Odom position
    OdomEntry entry = {};

    // Get data from CAN message
    entry.x = buffer_pop_int16(data, &idx) * 1000.0f;
    entry.y = buffer_pop_int16(data, &idx) * 1000.0f;
    entry.theta = buffer_pop_int16(data, &idx) * 1000.0f;

    // Add entry to history
    odomHistory[odom_seq % covar_samples] = entry;
  }
  else if (cmd_id == 37 && len >= 4) { // Odom velocity
    // Odom messages are sent position, then velocity
    // Get the most recent entry, put in by "odom position" branch, and parse this CAN data into it
    OdomEntry entry = odomHistory[odom_seq % covar_samples];
    entry.dx = buffer_pop_int16(data, &idx) * 1000.0f;
    entry.omega = buffer_pop_int16(data, &idx) * 1000.0f;

    nav_msgs::Odometry odom_msg;
    // Add header
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.seq = odom_seq++;
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;

    // Put data in message
    // We only use a subset of the total pose/twist pair, so some fields are permanently zero
    odom_msg.pose.pose.position.x = entry.x;
    odom_msg.pose.pose.position.y = entry.y;
    odom_msg.pose.pose.position.z = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sin(entry.theta / 2.0f); // Very simple way to make a quaternion around one axis
    odom_msg.pose.pose.orientation.w = cos(entry.theta / 2.0f);
    odom_msg.twist.twist.linear.x = entry.dx;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.linear.z = 0.0f;
    odom_msg.twist.twist.angular.x = 0.0f;
    odom_msg.twist.twist.angular.y = 0.0f;
    odom_msg.twist.twist.angular.z = entry.omega;

    // Calculate covariance and publish
    if (calculate_covariance(odom_msg.pose.covariance, odom_msg.twist.covariance)) {
      odom_pub.publish(odom_msg);
    }
  }
}

/**
 * Calculates the covariance matrices based on the odom history and stores the results in the provided arrays
 * Returns true if the returned covariance is valid, otherwise false
 */
bool calculate_covariance(boost::array<double, 36> &pose_mat, boost::array<double, 36> &twist_mat) {
  int count = std::min(odom_seq, covar_samples);
  if (count < 2) {
    return false; // Did not calculate covariance
  }
  OdomEntry avg = {};
  // Calculate averages
  for (int i = 0; i < count; i++) {
    avg.x += odomHistory[i].x;
    avg.y += odomHistory[i].y;
    avg.theta += odomHistory[i].theta;
    avg.dx += odomHistory[i].dx;
    avg.omega += odomHistory[i].omega;
  }
  avg.x /= count;
  avg.y /= count;
  avg.theta /= count;
  avg.dx /= count;
  avg.omega /= count;
  float avg_pose[] = {avg.x, avg.y, 0.0f, 0.0f, 0.0f, avg.theta};
  float avg_twist[] = {avg.dx, 0.0f, 0.0f, 0.0f, 0.0f, avg.omega};
  // Calculate covariance
  // See https://en.wikipedia.org/wiki/Covariance#Calculating_the_sample_covariance
  for (int i = 0; i < count; i++) {
    float item_pose[] = {odomHistory[i].x, odomHistory[i].y, 0.0f, 0.0f, 0.0f, odomHistory[i].theta};
    float item_twist[] = {odomHistory[i].dx, 0.0f, 0.0f, 0.0f, 0.0f, odomHistory[i].omega};
    for (int x = 0; x < 6; x++) {
      for (int y = 0; y < 6; y++) {
        int idx = 6*x + y;
        pose_mat[idx] = 0;
        twist_mat[idx] = 0;
        // Average mean error difference
        
        pose_mat[idx] += (item_pose[x] - avg_pose[x]) * (item_pose[y] - avg_pose[y]);
        twist_mat[idx] += (item_twist[x] - avg_twist[x]) * (item_twist[y] - avg_twist[y]);
        
        // Normalize
        pose_mat[idx] /= count - 1;
        twist_mat[idx] /= count - 1;
      }
    }
  }
  return true;
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

void unpack_frame(const can_msgs::Frame& frame, uint8_t* sys_id, uint8_t* cmd_id, uint8_t* data, uint8_t* len) {
  *sys_id = frame.id & 0xFF;
  *cmd_id = (frame.id >> 8) & 0xFF;
  *len = frame.dlc;
  memcpy(data, &(frame.data), *len);
}

int32_t buffer_pop_int32(uint8_t const* buffer, int* index) {
  int32_t buf = 0;
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

int16_t buffer_pop_int16(uint8_t const* buffer, int* index) {
  int16_t buf = 0;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int16(uint8_t* buffer, int* index, int16_t const value) {
  buffer[(*index)++] = value >> 8;
  buffer[(*index)++] = value;
}