/*
 * decawave_positioning.cpp
 * Higher level position calculation from decawave distance values
 * VERSION: 0.0
 * Last changed: 2019-12-04
 * Authors: Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainers: Amalia Schwartzwald <schw1818@umn.edu>
 * MIT License
 * Copyright (c) 2020 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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

class DecawavePositioning
{
public:
  // All of the received anchor data
  std::vector<anchor> anchors;
  double theta_0;
  double theta_1;

  


  DecawavePositioning(){

  }
  ~DecawavePositioning(){
    
  }
  void update_callback(const decawave::Range& range_msg){
  // ROS_INFO("Publishing..");

  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  range_msg.id
  range_msg.distance
  range_msg.position

  pose_msg.header.stamp = ros::Time::now();
  gps_pub.publish(pose_msg);
  }
  void positioning_calculations(){

  }
};
// ROS Node and Publishers
ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher gps_pub;

// ROS Topics
std::string gps_topic = "decawave/Range";//"odometry/gps";

int main(int argc, char** argv){
  // Params
  ros::param::get("~port_name",port_name);

  // Init this ROS node
  ros::init(argc, argv, "decawave_positioning");
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");

  // Subscribers
  ros::Subscriber sub = nh.subscribe("SUB_TOPIC_NAME", 1000, update_callback);

  // Publishers
  gps_topic = gps_topic + port_name;
  gps_pub = nh->advertise<decawave::Range>(gps_topic, 10);

  // Spin
  ros::spin();
}






// geometry_msgs/PoseWithCovarianceStamped.h
  // std_msgs/Header header
    // uint32 seq
    // time stamp
    // string frame_id
  // geometry_msgs/PoseWithCovariance pose
    // geometry_msgs/Pose pose
      // geometry_msgs/Point position
        // float64 x
        // float64 y
        // float64 z
      // geometry_msgs/Quaternion orientation
        // float64 x
        // float64 y
        // float64 z
        // float64 w
    // float64[36] covariance