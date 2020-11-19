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
//    estimate (geometry_msgs/PoseWithCovarianceStamped): "decawave"
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
double base_seperation_covar = 10.0;
double distance_mesurment_covar = 10.0;
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
std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err,double b,double b_err, double p1x, double p1y, double p2x, double p2y);

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
  estimate_pub = nh->advertise<nav_msgs::Odometry>("decawave/odom", 2);

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
    if (!tf_listener_->waitForTransform(map_frame_id, anchor0_frame_id, now, ros::Duration(0.5)) || 
        !tf_listener_->waitForTransform(map_frame_id, anchor1_frame_id, now, ros::Duration(0.5)) ||
        !tf_listener_->waitForTransform(robot_frame_id, decawave_frame_id, now, ros::Duration(0.5))) {
        return; // Can't find transforms
    }
    tf::StampedTransform anchor0_trs;
    tf::StampedTransform anchor1_trs;
    tf_listener_->lookupTransform(map_frame_id, anchor0_frame_id, now, anchor0_trs);
    tf_listener_->lookupTransform(map_frame_id, anchor1_frame_id, now, anchor1_trs);

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
    // todo: use positions from anchors relative to given frame?
    anchor0.position[0] = anchor0_trs.getOrigin().getX();
    anchor0.position[1] = anchor0_trs.getOrigin().getY();
    anchor0.position[2] = anchor0_trs.getOrigin().getZ();
    anchor1.position[0] = anchor1_trs.getOrigin().getX();
    anchor1.position[1] = anchor1_trs.getOrigin().getY();
    anchor1.position[2] = anchor1_trs.getOrigin().getZ();
    //std::cout << "Anchor " << anchor0.id << " dist: " << anchor0.distance << " Anchor " << anchor1.id << " dist: " << anchor1.distance <<std::endl;

    // Here be dragons
    // TODO: This badly needs a refactor
    if (anchor1.position[1] > anchor0.position[1]){
      covariance_vec= do_Math( anchor1.distance , distance_mesurment_covar , anchor0.distance , distance_mesurment_covar,
      sqrt( pow(  anchor1.position[1] -  anchor0.position[1] , 2) + pow(  anchor1.position[0] -  anchor0.position[0] , 2) ) , base_seperation_covar,
       anchor1.position[0],  anchor1.position[1],  anchor0.position[0],  anchor0.position[1] );
    }
    else {//anchor 0 is p1
      covariance_vec= do_Math( anchor0.distance , distance_mesurment_covar , anchor1.distance , distance_mesurment_covar,
      sqrt( pow(  anchor1.position[1] -  anchor0.position[1] , 2) + pow(  anchor1.position[0] -  anchor0.position[0] , 2) ) , base_seperation_covar,
      anchor0.position[0],   anchor0.position[1],  anchor1.position[0],  anchor1.position[1] );
    }

    //handle no val
    if (covariance_vec.size() < 36) {
      ROS_INFO("do_Math failed");
      return;
    }
    boost::array<double, 36> covariance;
    for(int i = 0; i<36; i++){
      covariance[i] = covariance_vec[i];
    }

    nav_msgs::Odometry odom_msg;
    // Add header
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.seq = seq++;
    odom_msg.header.frame_id = map_frame_id;
    odom_msg.child_frame_id = robot_frame_id; // we don't actually specify orientation but should do this just because

    // Put data in message
    // We only use a subset of the total pose/twist pair, so some fields are permanently zero
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = 0.0f; 
    odom_msg.pose.pose.orientation.w = 1.0f;
    odom_msg.twist.twist.linear.x = 0.0f;
    odom_msg.twist.twist.linear.y = 0.0f;
    odom_msg.twist.twist.linear.z = 0.0f;
    odom_msg.twist.twist.angular.x = 0.0f;
    odom_msg.twist.twist.angular.y = 0.0f;
    odom_msg.twist.twist.angular.z = 0.0f;

    //take x and y off the back and put them in the pose
    float y = covariance_vec[37];
    covariance_vec.pop_back();
    float x = covariance_vec[36];
    covariance_vec.pop_back();
    float z = anchor0.position[2]; // Assume decawaves are all planar

    // Transform pose into robot position in map frame
    tf::Vector3 posOut(x,y,z);
    tf::StampedTransform trs;
    tf_listener_->lookupTransform(decawave_frame_id, robot_frame_id, now, trs);
    tf::Vector3 posRobot = trs * posOut;
    odom_msg.pose.pose.position.x = posRobot.getX();
    odom_msg.pose.pose.position.y = posRobot.getY();
    odom_msg.pose.pose.position.z = posRobot.getZ();

    odom_msg.pose.covariance = covariance;
    // This system doesn't provide any orientation estimate, so just set to 0
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 0.0;

    estimate_pub.publish(odom_msg);
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

//ASSUMES x,y is inside bounds. assumes p1 y is greater than p2 y
*/
std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err,double b,double b_err, double p1x, double p1y, double p2x, double p2y){
  //protect triangle inequality
  if ( abs(d0-d1) > b){
    //std::cout << "vec: " << d0 << " " << d1 << " " << b << "\n";
    std::vector<double> bad_val={0};
    return bad_val;
  } 
  //calc cos theta
  double cos_theta = (pow(b,2) + pow(d0,2) - pow(d1,2) )/(2*b*d0);
  double b_sqr_err= 2*b_err;
  double d0_sqr_err= 2*d0_err;
  double d1_sqr_err= 2*d1_err;
  double top_err = sqrt( ( pow(b_sqr_err,2) + pow( d0_sqr_err, 2 ) + pow( d1_sqr_err, 2) ) );
  double theta_err = sqrt( pow(b_err/b , 2) + pow( top_err / (pow(b,2) + pow(d0,2) - pow(d1,2)) ,2) + pow(d0_err/d0 , 2));

  double angle = acos(cos_theta);
  double angle_diff = acos(cos_theta+theta_err);

  if(angle_diff<angle){
    angle_diff= angle - acos(cos_theta-theta_err);
  }else{
    angle_diff = angle_diff-angle;
  }
  double sin_theta = sin(angle);

  //calc x and y covarience
  double x_co = d0*tan(angle_diff);
  double y_co = d0_err;

  //unit vetors in y and x directions respecivly
  //rotate b by theta
  std::vector<double> v2={(p2x-p1x)*cos_theta - (p2y-p1y)*sin_theta , (p2x-p1x)*sin_theta + (p2y-p1y)*cos_theta };
  double v2_len= sqrt(pow(v2[0], 2)+pow(v2[1], 2));
  //std::cout << "vec: " << d0 << " " << d1 << " " << b << "\n";
  v2[0]=v2[0]/v2_len;
  v2[1]=v2[1]/v2_len;
  std::vector<double> v1={v2[1], -v2[0]};

  std::vector<double> r = {v1[0], v1[1], v2[0], v2[1]};

  //covariance is R C (R transpose)
  //set c1 =R * [dxx , 0, 0 , dyy]
  std::vector<double> c1={ x_co*r[0], y_co*r[1], x_co*r[2], y_co*r[3]};

  std::vector<double> covar_mat={0.0, 0.0, 0.0, 0.0};
  //mult by r transpose
  covar_mat[0]= c1[0]*r[0] + c1[1]*r[1];
  covar_mat[1]= c1[0]*r[2] + c1[1]*r[3];
  covar_mat[2]= c1[2]*r[0] + c1[3]*r[1];
  covar_mat[3]= c1[2]*r[2] + c1[3]*r[3];

  //calc x,y
  double x = p1x+v2[0]*d0;
  double y = p1y+v2[1]*d0;

  if (isnan(covar_mat[0] + covar_mat[1] + covar_mat[2] + covar_mat[3])) {
    std::vector<double> covar = {};
    return covar; // error return
  }

  std::vector<double> covariance=
    { covar_mat[0], covar_mat[1], 0.0, 0.0, 0.0, 0.0,
      covar_mat[2], covar_mat[3], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 10000.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 10000.0,
      x, y
    };

  return covariance;
}

void gazebo_joint_callback(const gazebo_msgs::LinkStates& msg) {
  if (is_sim) {
    static_cast<decawave::DecawaveSim*>(piTag)->gazebo_joint_callback(msg);
  }
}
