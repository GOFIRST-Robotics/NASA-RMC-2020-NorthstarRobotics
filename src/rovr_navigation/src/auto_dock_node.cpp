/*
 * auto_dock_node.cpp 
 * Navigation to align with the trough
 * VERSION: 1.0
 * Last changed: 2020-02-08
 * Authors: Dan Evenson <evens352@umn.edu>
 * MIT License
 * Copyright (c) 2020 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Native_Libs
#include <string>
#include <math.h>

// Subscribers (inputs)
//    fid (fiducial_msgs/FiducialTransformArray): "dock/fid"  
//      aruco marker fiducial 
//    dock_enable (std_msgs/Bool): "dock/enable"
//      enables docking or disable docking process
//    pose (geometry_msgs/PoseWithCovarianceStamped): "dock/pose"
//      aruco marker pose

// Publishers (outputs)
//    cmd_vel (geometry_msgs/TwistStamped): "cmd_vel"
//      cmd_vel from auto dock
//    state (std_msgs/Int16): "dock/state"
//      state of auto dock

// Parameters (settings)
//    frequency (double): default= 10.0 [Hz]
//    rotating_speed (double): default= 1.5 [rad/s]
//    docking_angle_error_deg (double): default= 10.0 [deg]
//    docking_distance_scale (double): default= 0.5 [0,1]
//    docking_distance_threshold (double): default= 0.05 [m]
//    approach_speed (double): default = 0.5 [m/s]
//    use_pose_or_fiducials (boolean): default = true

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher cmd_vel_pub;
ros::Publisher state_pub;

// ROS Callbacks
void fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void dock_enable_callback(const std_msgs::Bool::ConstPtr& msg);
void timer_callback(const ros::TimerEvent&);

// ROS Params
double frequency = 10.0;
double rotating_speed = 1.5;
double docking_angle_error_deg = 10.0;
double docking_distance_scale = 0.5;
double docking_distance_threshold = 0.05;
double approach_speed = 2.0;
bool use_pose_or_fiducials = true;

enum State {
  ST_APPROACH,
  ST_DOCKED,
  ST_CENTERING,
  ST_DISABLED,
};

State st_approach_callback(geometry_msgs::TwistStamped& cmd_vel_msg);
State st_docked_callback(geometry_msgs::TwistStamped& cmd_vel_msg);
State st_centering_callback(geometry_msgs::TwistStamped& cmd_vel_msg);
State st_disabled_callback(geometry_msgs::TwistStamped& cmd_vel_msg);

// Global_Vars
State state = ST_DISABLED;
unsigned int seq = 0;
bool fid_is_in_view = false; // Only changed in fid_callback
double fid_angle = 0.0;
double fid_distance = 0.0;

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, "auto_dock_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");
  ros::Subscriber fid_sub;
  ros::Subscriber pose_sub;
  // Params
  pnh->getParam("frequency", frequency);
  pnh->getParam("rotating_speed", rotating_speed);
  pnh->getParam("docking_distance_threshold", docking_distance_threshold);
  pnh->getParam("docking_distance_scale", docking_distance_scale);
  pnh->getParam("approach_speed", approach_speed);
  pnh->getParam("use_pose_or_fiducials", use_pose_or_fiducials);
  //bool use_pose_or_fiducials = false;
  
  // Subscribers
  ros::Subscriber dock_enable = nh->subscribe("dock/enable", 1, dock_enable_callback);
  ros::Timer timer = nh->createTimer(ros::Duration(1.0/frequency), timer_callback);
  if(use_pose_or_fiducials){
    pose_sub = nh->subscribe("dock/pose", 1, pose_callback);
  }else{
    fid_sub = nh->subscribe("dock/fid", 1, fid_callback);
  } 
 
  
  
    
  
  
  // Publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
  state_pub = nh->advertise<std_msgs::Int16>("dock/state", 1);
  
  
  // Spin
  ros::spin();
}

void timer_callback(const ros::TimerEvent&){
geometry_msgs::TwistStamped cmd_vel_msg;
cmd_vel_msg.header.seq = seq++;
cmd_vel_msg.header.stamp = ros::Time::now();
cmd_vel_msg.header.frame_id = "base_link";
switch(state){
  case ST_DISABLED : 
    state = st_disabled_callback(cmd_vel_msg);
    break;
  case ST_CENTERING :
    state = st_centering_callback(cmd_vel_msg);
    cmd_vel_pub.publish(cmd_vel_msg);
    break;
  case ST_APPROACH :
    state = st_approach_callback(cmd_vel_msg);
    cmd_vel_pub.publish(cmd_vel_msg);
    break; 
  case ST_DOCKED :
    state = st_docked_callback(cmd_vel_msg);
    cmd_vel_pub.publish(cmd_vel_msg);
    break;
  default :
    state = ST_DISABLED;
    ROS_ERROR("Auto docking wrong state: %i", state);
    return;   
}
std_msgs::Int16 state_msg; 
state_msg.data = state;
state_pub.publish(state_msg);
}

State st_approach_callback(geometry_msgs::TwistStamped& cmd_vel_msg){
  //enable aruco detection
  double percent_error = fid_angle / 
    ((fid_distance * docking_distance_scale) + docking_angle_error_deg);
  double vel_scale = 1.0 - (1.0 / (1.0 + (fid_distance / (docking_distance_scale + 0.00001)))); 
  cmd_vel_msg.twist.angular.z = percent_error * rotating_speed;
  cmd_vel_msg.twist.linear.x = approach_speed * vel_scale;
  if (fid_distance > docking_distance_threshold){
    return ST_APPROACH;
  }else{
    return ST_DOCKED;
  }
}

State st_docked_callback(geometry_msgs::TwistStamped& cmd_vel_msg){
  // Disbale aruco detection
  // Set cmd_vel to 0
  cmd_vel_msg.twist.linear.x = 0.0;
  cmd_vel_msg.twist.angular.z = 0.0;
  return ST_DISABLED;
}

State st_centering_callback(geometry_msgs::TwistStamped& cmd_vel_msg){
  // Enable aruco detection
  // Find and center on markers 
  if(!fid_is_in_view){
    // Turn in place mode
    cmd_vel_msg.twist.linear.x = 0.0;
    cmd_vel_msg.twist.angular.z = rotating_speed;
    return ST_CENTERING;
  }
  double percent_error = fid_angle / 
    ((fid_distance * docking_distance_scale) + docking_angle_error_deg);
  cmd_vel_msg.twist.linear.x = 0.0;
  cmd_vel_msg.twist.angular.z = (fid_angle > 0) ? -rotating_speed : rotating_speed;
  if(percent_error > 1.0 || percent_error < -1.0) {
    return ST_CENTERING;
  }
  // Else
    return ST_APPROACH;
}

State st_disabled_callback(geometry_msgs::TwistStamped& cmd_vel_msg){
  return ST_DISABLED;
}

void fid_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  fid_is_in_view = !(msg->transforms.empty());
  if(!fid_is_in_view) return;
  
  fid_angle = 0.0;
  fid_distance = 0.0;
  for (auto & fid_tf : msg->transforms) {
    fid_angle += tf::getYaw(fid_tf.transform.rotation);
    fid_distance += sqrt(pow(fid_tf.transform.translation.x,2) +        pow(fid_tf.transform.translation.y,2.0));        
  }
  fid_angle /= msg->transforms.size();
  fid_distance /= msg->transforms.size();
  
}
  
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  fid_angle = 0.0;
  fid_distance = 0.0; 
  fid_angle = tf::getYaw(msg->pose.pose.orientation);
  fid_distance = sqrt(pow(msg->pose.pose.orientation.x,2) + pow(msg->pose.pose.orientation.y, 2));
}

void dock_enable_callback(const std_msgs::Bool::ConstPtr& msg){
  if(msg->data){
    state = ST_CENTERING;
  }else{
    state = ST_DISABLED;
  }
}
  
  
  
  
  
  



