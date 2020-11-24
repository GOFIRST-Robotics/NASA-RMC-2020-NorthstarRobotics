/*
 * dual_decawave_node.cpp
 * Combine two decawave position readings into 3DoF pose
 * VERSION: 1.0
 * Authors: Julia Schatz <schat127@umn.edu>
 * Maintainers: Julia Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2020 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Native_Libs
#include <string>
#include <time.h>
#include <functional>

// Typedefs
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> ApproxPoseSyncPolicy;
typedef message_filters::Synchronizer<ApproxPoseSyncPolicy> ApproxPoseSync;

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher estimate_pub;

// ROS Callbacks
void decawave_callback(const geometry_msgs::PoseWithCovarianceStamped& dw0, const geometry_msgs::PoseWithCovarianceStamped& dw1);

// ROS Params
std::string decawave0_frame_id = "decawave0_link";
std::string decawave1_frame_id = "decawave1_link";
std::string robot_frame_id = "base_link";
std::string map_frame_id = "map";

// Global_Vars
tf::TransformListener * tf_listener_;
int seq = 0;

int main(int argc, char** argv){
  // Init this ROS node
  ros::init(argc, argv, "dual_decawave_node");
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");
  tf_listener_ = new tf::TransformListener();

  // Params
  pnh->getParam("decawave0_frame_id", decawave0_frame_id);
  pnh->getParam("decawave1_frame_id", decawave1_frame_id);

  // Subscribers
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> decawave0_sub(*nh, "decawave0/pose", 5);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> decawave1_sub(*nh, "decawave1/pose", 5);

  ApproxPoseSync sync(ApproxPoseSyncPolicy(10), decawave0_sub, decawave1_sub);
  sync.registerCallback(decawave_callback);

  // Publishers
  estimate_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("decawave/pose", 2);

  // Spin
  ros::spin();
}

void decawave_callback(const geometry_msgs::PoseWithCovarianceStamped& dw0, const geometry_msgs::PoseWithCovarianceStamped& dw1) {
  ros::Time now = ros::Time::now();
  if (!tf_listener_->waitForTransform(decawave0_frame_id, robot_frame_id, now, ros::Duration(0.5)) || 
      !tf_listener_->waitForTransform(decawave1_frame_id, robot_frame_id, now, ros::Duration(0.5))) {
      ROS_WARN("Couldn't find necessary transforms for dual-DW node");
      return; // Can't find transforms
  }
  // todo: make sure these are in the same frame
  double dw0_xstdev = sqrt(dw0.pose.covariance[0]);
  double dw0_ystdev = sqrt(dw0.pose.covariance[7]);
  double dw1_xstdev = sqrt(dw1.pose.covariance[0]);
  double dw1_ystdev = sqrt(dw1.pose.covariance[7]);
  double dw0_x = dw0.pose.pose.position.x;
  double dw0_y = dw0.pose.pose.position.y;
  double dw1_x = dw1.pose.pose.position.x;
  double dw1_y = dw1.pose.pose.position.y;
  // Estimate centerpoint of decawaves
  // Combine x/y in weighted average
  double w0_x = dw0_xstdev;
  double w1_x = dw1_xstdev;
  double w0_y = dw0_ystdev;
  double w1_y = dw1_ystdev;
  double est_x = (dw0_x * w0_x + dw1_x * w1_x) / (w0_x + w1_x);
  double est_y = (dw0_y * w0_y + dw1_y * w1_y) / (w0_y + w1_y);

  // todo: convert this to base_link position if decawaves' mean isn't at base_link

  std::vector<double> covar = {
    dw0_xstdev * dw1_xstdev, 0, 0, 0, 0, 0,
    0, dw0_ystdev * dw1_ystdev, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0
  };

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
    // Add header
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.seq = seq++;
    pose_msg.header.frame_id = dw0.header.frame_id;

    // Put data in message
    
    pose_msg.pose.pose.position.x = est_x;
    pose_msg.pose.pose.position.y = est_y;
    pose_msg.pose.pose.position.z = dw0.pose.pose.position.z;

    boost::array<double, 36> covariance;
    for(int i = 0; i<36; i++){
      covariance[i] = covar[i];
    }
    pose_msg.pose.covariance = covariance;
    // This system doesn't provide any orientation estimate, so just set to 0
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 0.0;

    estimate_pub.publish(pose_msg);
}