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
      !tf_listener_->waitForTransform(decawave1_frame_id, robot_frame_id, now, ros::Duration(0.5)) ||
      !tf_listener_->waitForTransform(map_frame_id, dw0.header.frame_id, now, ros::Duration(0.5)) ||
      !tf_listener_->waitForTransform(map_frame_id, dw1.header.frame_id, now, ros::Duration(0.5))) {
      ROS_WARN("Couldn't find necessary transforms for dual-DW node");
      return; // Can't find transforms
  }
  tf::StampedTransform dw0_base_trs;
  tf_listener_->lookupTransform(robot_frame_id, decawave0_frame_id, now, dw0_base_trs);
  tf::StampedTransform dw_dw_trs;
  tf_listener_->lookupTransform(decawave1_frame_id, decawave0_frame_id, now, dw_dw_trs);
  tf::Vector3 dw_dw_origin_true = dw_dw_trs.getOrigin();
  double dw_dw_len = dw_dw_origin_true.length();
  double angle_true = atan2(dw_dw_origin_true.getY(), dw_dw_origin_true.getX());

  // todo: make sure these are in the same frame
  double dw0_xstdev = sqrt(dw0.pose.covariance[0]);
  double dw0_ystdev = sqrt(dw0.pose.covariance[7]);
  double dw1_xstdev = sqrt(dw1.pose.covariance[0]);
  double dw1_ystdev = sqrt(dw1.pose.covariance[7]);
  double dw0_x = dw0.pose.pose.position.x;
  double dw0_y = dw0.pose.pose.position.y;
  double dw1_x = dw1.pose.pose.position.x;
  double dw1_y = dw1.pose.pose.position.y;
  // Estimate distance between decawanes
  // Combine x/y in weighted average
  // Weights are chosen so that high stdevs are weighted low, while very small ones still don't dominate
  double w0_x = 1/sqrt(dw0_xstdev);
  double w1_x = 1/sqrt(dw1_xstdev);
  double w0_y = 1/sqrt(dw0_ystdev);
  double w1_y = 1/sqrt(dw1_ystdev);
  double est_x = ((dw0_x * w0_x + dw1_x * w1_x) / (w0_x + w1_x) - dw0_x);
  double est_y = ((dw0_y * w0_y + dw1_y * w1_y) / (w0_y + w1_y) - dw1_x);

  double len_guess = sqrt(est_x*est_x + est_y*est_y);
  double angle_guess = atan2((dw1_y - dw0_y), (dw1_x - dw0_x));
  
  // Guess position of base_link based on known difference between decawaves
  tf::Vector3 translation = dw0_base_trs.getOrigin();
  double len_ratio = len_guess / dw_dw_len / 2;
  translation.setX(translation.getX() * len_ratio);
  translation.setY(translation.getY() * len_ratio);
  tf::Pose dw0_pose;
  tf::poseMsgToTF(dw0.pose.pose, dw0_pose);
  tf::Transform guess_trs = tf::Transform(tf::Quaternion::getIdentity(), -translation);
  tf::Pose base_link_pose = dw0_pose;
  base_link_pose.setRotation(tf::Quaternion(tf::Vector3(0,0,1), angle_guess - angle_true + M_PI));
  base_link_pose *= guess_trs;


  std::vector<double> covar = {
    dw0_xstdev * dw1_xstdev, 0, 0, 0, 0, 0,
    0, dw0_ystdev * dw1_ystdev, 0, 0, 0, 0,
    0, 0, 0, 0, 0, M_PI * (exp((len_ratio-1) * (len_ratio-1)) - 1),
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
    tf::poseTFToMsg(base_link_pose, pose_msg.pose.pose);\

    boost::array<double, 36> covariance;
    for(int i = 0; i<36; i++){
      covariance[i] = covar[i];
    }
    pose_msg.pose.covariance = covariance;

    estimate_pub.publish(pose_msg);
}