/*
 * decawave_node.cpp
 * ROS interface to Decawave class
 * VERSION: 1.0
 * Last changed: 2020-03-08
 * Authors: Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainers: Amalia Schwartzwald <schw1818@umn.edu>
 * MIT License
 * Copyright (c) 2020 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/LinkStates.h>

// Native_Libs
#include <string>
#include <time.h>
#include <functional>

// Custom_Libs
#include "decawave/decawave.h"
#include "decawave/Range.h"

// Subscribers (inputs)
//    update_timer (Timer)
//      Update loop for reading / querying Decawave

// Publishers (outputs)
//    estimate (geometry_msgs/PoseWithCovarianceStampedStamped): "decawave"
//      Calculated x/y position of robot

// Parameters (settings)
//    frequency (double): default=10.0; The frequency of the update loop
//    base_separation_covar (double): default=10.0; The variance of the distance between anchors?
//    distance_measurement_covar (double): default=10.0; The variance of the distance measurement?
//    port_name (string): default=/dev/ttyACM0; The serial port the decawave tag is connected to 
//    anchor0_frame (string): default=anchor0; The frame whose origin defines the position of anchor tag 0
//    anchor1_frame (string): default=anchor1; The frame whose origin defines the position of anchor tag 1
//    anchor0_dw_id (int): default=0; The lowest 2 bits of the anchor 0 tag ID, expressed in decimal
//    anchor1_dw_id (int): default=0; The lowest 2 bits of the anchor 1 tag ID, expressed in decimal
//    decawave_frame (string): default=decawave; The frame of the decawave tag
//    robot_frame (string): default=decawave; The frame to express the pose estimate of
//    map_frame (string): default=map; The fixed world frame
//    is_sim (bool): default=false; Whether we are in a gazebo environment or not
//    gazebo_prefix (string): default=rovr
//    sim_dist_stdev (double) default=0.1; Standard deviation of simulated distance measurements (meters)

// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher estimate_pub;

// ROS Callbacks
void update_callback(const ros::TimerEvent&);
void gazebo_joint_callback(const gazebo_msgs::LinkStates& msg);

// ROS Params
double frequency = 10.0;
double base_seperation_covar = 0.1;
double distance_mesurment_covar = 0.1;
std::string port_name = "/dev/ttyACM0";
std::string anchor0_frame_id = "anchor0";
std::string anchor1_frame_id = "anchor1";
int anchor0_dw_id = 0;
int anchor1_dw_id = 0;
std::string decawave_frame_id = "decawave";
std::string robot_frame_id = "base_link";
std::string map_frame_id = "map";
bool is_sim = false;
std::string gazebo_prefix = "rovr";
double sim_dist_stdev = 0.1;

// Global_Vars
decawave::IDecawave *piTag;
tf::TransformListener * tf_listener_;
int seq = 0;

// Utility Functions
std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err, double b);

int main(int argc, char** argv){
  // Init this ROS node
  ros::init(argc, argv, "decawave_node");
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");
  tf_listener_ = new tf::TransformListener();

  // Params
  pnh->getParam("port_name", port_name);
  pnh->getParam("base_seperation_covar",base_seperation_covar);
  pnh->getParam("distance_measurement_covar",distance_mesurment_covar);
  pnh->getParam("anchor0_frame", anchor0_frame_id);
  pnh->getParam("anchor1_frame", anchor1_frame_id);
  pnh->getParam("anchor0_dw_id", anchor0_dw_id);
  pnh->getParam("anchor1_dw_id", anchor1_dw_id);
  pnh->getParam("decawave_frame", decawave_frame_id);
  pnh->getParam("robot_frame", robot_frame_id);
  pnh->getParam("map_frame", map_frame_id);
  pnh->getParam("is_sim", is_sim);
  pnh->getParam("gazebo_prefix", gazebo_prefix);
  pnh->getParam("sim_dist_stdev", sim_dist_stdev);

  // Init Decawave
  if (is_sim) {
    piTag = new decawave::DecawaveSim(map_frame_id, decawave_frame_id, anchor0_dw_id, anchor1_dw_id, anchor0_frame_id, anchor1_frame_id, gazebo_prefix, robot_frame_id, sim_dist_stdev, tf_listener_);
  }
  else {
    piTag = new decawave::Decawave(port_name);
  }

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);
  ros::Subscriber joint_sub = nh->subscribe("/gazebo/link_states", 5, gazebo_joint_callback);

  // Publishers
  estimate_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("decawave/pose", 2);

  // Spin
  ros::spin();
}


void update_callback(const ros::TimerEvent&){
  // ROS_INFO("Publishing..");
  decawave::Range msg;

  // update decawave data
  std::vector<decawave::Anchor> anchors = piTag->updateSamples();

  if (anchors.size() >= 2) {
    std::vector<double> covariance_vec;
    // get xyz positions of each anchor from tf
    ros::Time now = ros::Time::now();
    if (!tf_listener_->waitForTransform(anchor1_frame_id, anchor0_frame_id, now, ros::Duration(0.5))) {
        ROS_WARN("Couldn't find necessary transforms for DW node");
        return; // Can't find transforms
    }
    tf::StampedTransform anchor01_trs;
    tf_listener_->lookupTransform(anchor1_frame_id, anchor0_frame_id, now, anchor01_trs);
    tf::Vector3 trs_origin = anchor01_trs.getOrigin();
    if (abs(trs_origin.getY()) > 1e-2 || abs(trs_origin.getZ()) > 1e-2) {
      ROS_WARN("Anchor frames set improperly- they must be the same except for in anchor0's X frame");
    }

    // Find anchors by id
    decawave::Anchor anchor0;
    decawave::Anchor anchor1;
    int counted = 0;
    for (decawave::Anchor anc : anchors) {
      if (anc.id == anchor0_dw_id) {
        anchor0 = anc;
        counted++;
      }
      if (anc.id == anchor1_dw_id) {
        anchor1 = anc;
        counted++;
      }
    }
    if (counted < 2) {
      ROS_INFO("Couldn't find correct anchors");
      return;
    }

    //std::cout << "Anchor " << anchor0.id << " dist: " << anchor0.distance << " Anchor " << anchor1.id << " dist: " << anchor1.distance <<std::endl;

    // Here be dragons
    // TODO: This badly needs a refactor
    covariance_vec= do_Math( anchor1.distance , distance_mesurment_covar , anchor0.distance , distance_mesurment_covar, trs_origin.getX());

    //handle no val
    if (covariance_vec.size() < 36) {
      ROS_INFO("do_Math failed");
      return;
    }
    boost::array<double, 36> covariance;
    for(int i = 0; i<36; i++){
      covariance[i] = covariance_vec[i];
    }

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    // Add header
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.seq = seq++;
    pose_msg.header.frame_id = anchor0_frame_id;

    // Put data in message

    //take x and y off the back and put them in the pose
    float y = covariance_vec[37];
    covariance_vec.pop_back();
    float x = covariance_vec[36];
    covariance_vec.pop_back();
    float z = 0; // Assume decawaves are all planar
    
    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = z;

    pose_msg.pose.covariance = covariance;
    // This system doesn't provide any orientation estimate, so just set to 0
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    estimate_pub.publish(pose_msg);
  }
}

//gives flattened 6x6 covarience matrix, x ,y in the same frame as the endpoints of b
/*
      (x,y)
        |\
        | \
        |  \
      d0|   \ d1
        |    \
        |     \
  theta --------
      p1   b     p2

*/
std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err,double b){

  // Calculate circ-circ intersection
  double x = -(b*b - d0*d0 + d1*d1) / (2*b);
  double y = +sqrt(d1*d1 - x*x); // Take positive square root
  // Rough standard deviation calculation
  double xerr = (2 * d0 * d0_err + 2 * d1 * d1_err + d0_err*d0_err + d1_err*d1_err) / (2*b);
  double yerr = (2 * d1 * d1_err + d1_err * d1_err + 2*x*xerr + xerr*xerr) / (2*y);
  // Rotate back into real frame

  std::vector<double> covar = {
    xerr*xerr, 0, 0, 0, 0, 0,
    0, yerr*yerr, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, x, y
  };
  return covar;
}

void gazebo_joint_callback(const gazebo_msgs::LinkStates& msg) {
  if (is_sim) {
    static_cast<decawave::DecawaveSim*>(piTag)->gazebo_joint_callback(msg);
  }
}
