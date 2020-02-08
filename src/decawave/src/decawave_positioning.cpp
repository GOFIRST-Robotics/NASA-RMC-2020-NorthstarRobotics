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
#include <math.h>

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
  ros::param::get("~port_name", port_name);

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
std::vector<float> do_Math(float d0, float d0_err,float d1, float d1_err,float b,float b_err, float p1x, float p1y, float p2x, float p2y){
  //calc cos theta
  float cos_theta = (pow(b,2) + pow(d0,2) - pow(d1,2) )/(2*b*d0);
  float b_sqr_err= 2*b_err;
  float d0_sqr_err= 2*d0_err;
  float d1_sqr_err= 2*d1_err;
  float top_err = sqrt( ( pow(b_sqr_err,2) + pow( d0_sqr_err, 2 ) + pow( d1_sqr_err, 2) ) );
  float theta_err = sqrt( pow(b_err/b , 2) + pow( top_err / (pow(b,2) + pow(d0,2) - pow(d1,2)) ,2) + pow(d0_err/d0 , 2);

  float angle = acos(cos_theta);
  float angle_diff = acos(cos_theta+theta_err));

  if(angle_diff<angle){
    angle_diff= angle - acos(cos_theta-theta_err);
  }else{
    angle_diff = angle_diff-angle;
  }
  float sin_theta = sin(angle);

  //calc x and y covarience
  float x_co = d0*tan(angle_diff);
  float y_co = d0_err;

  //unit vetors in y and x directions respecivly
  std::vector<float> v2={(p2x-p1x)*cos_theta - (p2y-p1y)*sin_theta , (p2x-p1x)*sin_theta + (p2y-p1y)*cos_theta };
  float v2_len= sqrt(pow(v2[0], 2)+pow(v2[1], 2));
  v2[0]=v2[0]/len;
  v2[1]=v2[1]/len;
  std::vector<float> v1={v2[1], -v2[0]};

  std::vector<float> r = {v1[0], v1[1], v2[0], v2[1]};

  //covariance is R C (R transpose)
  //set c1 =R * [dxx , 0, 0 , dyy]
  std::vector<float> c1={ x_co*r[0], y_co*r[1], x_co*r[2], y_co*r[3]};
  //mult by r transpose
  c1[0]= c1[0]*r[0] + c1[1]*r[1];
  c1[1]= c1[0]*r[2] + c1[1]*r[3];
  c1[2]= c1[2]*r[0] + c1[3]*r[1];
  c1[3]= c1[2]*r[2] + c1[3]*r[3];

  //calc x,y
  float x = p1x+v2[0]*d0;
  float y = p1y+v2[1]*d0;

  std::vector<float> covariance=
    { c1[0], c1[1], 0.0, 0.0, 0.0, 0.0,
      c1[2], c1[3], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 10000.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 10000.0,
      x, y
    };

  return covariance
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
