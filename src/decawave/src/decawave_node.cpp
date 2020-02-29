/*
 * decawave_node.cpp
 * ROS interface to Decawave class
 * VERSION: 1.0
 * Last changed: 2019-04-01
 * Authors: Amalia Schwartzwald <schw1818@umn.edu>
 * Maintainers: Amalia Schwartzwald <schw1818@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Native_Libs
#include <string>
#include <time.h>

// Custom_Libs
#include "decawave/decawave.h"
#include "decawave/Range.h"

// Subscribers (inputs)
//    update_timer (Timer)
//      Update loop for reading / querying Decawave
//    sub_name1 (sub_name1_type): sub_name1_TOPIC_VALUE
//      sub_name1_desc

// Publishers (outputs)
//    dw_pub (nav_msgs/Odometry): "odometry/gps"
//      A simulated local position as if GPS UTM

// Parameters (settings)


//    frequency (double): default=50.0
//      The update frequency of the update loop
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)

// ROS Node and Publishers

ros::NodeHandle * nh;
ros::NodeHandle * pnh;
ros::Publisher dw_pub;
ros::Publisher estimate_pub;

// ROS Topics
std::string gps_topic = "decawave/Range";//"odometry/gps";

// ROS Callbacks
void update_callback(const ros::TimerEvent&);

std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err,double b,double b_err, double p1x, double p1y, double p2x, double p2y);
// ROS Params
double frequency = 5.0;
double base_seperation_covar=10.0;
double distance_mesurment_covar=10.0;
std::string port_name;
int seq=0;
// for(int i = 0; i < argc; ++i){
// int port_num = atoi(argv[1]);
// }
// Global_Vars
decawave::Decawave *piTag;

int main(int argc, char** argv){
  // Init this ROS node
  ros::init(argc, argv, "decawave_1");
  nh = new ros::NodeHandle("");
  pnh = new ros::NodeHandle("~");

  // Params
  ros::param::get("/decawave_node/port_name",port_name);
  ros::param::get("/decawave_node/base_seperation_covar",base_seperation_covar);
  ros::param::get("/decawave_node/distance_mesurment_covar",distance_mesurment_covar);

  // Init Decawave

  piTag = new decawave::Decawave(port_name);

  // Subscribers
  ros::Timer update_timer = nh->createTimer(ros::Duration(1.0/frequency), update_callback);

  // Publishers
  gps_topic = gps_topic;
  dw_pub = nh->advertise<decawave::Range>(gps_topic, 10);
  estimate_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("decawave", 2);
  // Spin
  ros::spin();
}


void update_callback(const ros::TimerEvent&){
  // ROS_INFO("Publishing..");
  decawave::Range msg;

  // update decawave data
  std::vector<decawave::Anchor> anchors = piTag->updateSamples();
  ROS_INFO("Samples updated");
  std::string frame_id = port_name;
  std::cout << "numanchors" << anchors.size() << "/n";
  decawave::Anchor m_anchor;
  for (int i=0; i<anchors.size(); i++) {
    m_anchor=anchors[i];//get new anchor
    msg.distance = ((float)m_anchor.distance)/1000;
    msg.distance_quality = m_anchor.distance_quality;
    msg.quality_factor=m_anchor.quality_factor;
    //position in (x,y,z)
    msg.position={((float)m_anchor.position[0])/1000,
      ((float)m_anchor.position[1])/1000,
      ((float)m_anchor.position[2])/1000};
    // fill out message header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "decawave_" + frame_id;//"decawave_" + port_num;
    msg.child_frame_id = "decawave2_link"; //change to correct part
    msg.id=(int)m_anchor.id;
    //dw_pub.publish(msg);
    i++;
  }
/*
  decawave::Anchor anchor_1 = {1, 12000, 100, {0,10000,1}, 100};
  decawave::Anchor anchor_2 = {2, 12900, 100, {0,9000,1}, 100};
  std::vector<decawave::Anchor> anchors={ anchor_1, anchor_2};
*/
  //check if 2 anchors are present.
  if(anchors.size()>1){
    ROS_INFO("Anchors seen");
    //compare y vals
    std::vector<double> covariance_vec;
    //if anchor 1 is p1
    anchors[0].position[0]=0;
    anchors[0].position[1]=9000;
    anchors[1].position[0]=0;
    anchors[1].position[1]=10000;
    std::cout << "an1pos: " <<anchors[0].distance << " " << anchors[1].distance <<"\n";

    if (anchors[1].position[1] >anchors[0].position[1]){
      covariance_vec= do_Math( (double)anchors[1].distance/1000 , distance_mesurment_covar , (double)anchors[0].distance/1000 , distance_mesurment_covar,
      sqrt( pow( (double) anchors[1].position[1] - (double) anchors[0].position[1] , 2) + pow( (double) anchors[1].position[0] - (double) anchors[0].position[0] , 2) )/1000 , base_seperation_covar,
      (double) anchors[1].position[0]/1000, (double) anchors[1].position[1]/1000, (double) anchors[0].position[0]/1000, (double) anchors[0].position[1]/1000 );
    }else{//anchor 0 is p1
      covariance_vec= do_Math( (double)anchors[0].distance/1000 , distance_mesurment_covar , (double)anchors[1].distance/1000 , distance_mesurment_covar,
      sqrt( pow( (double) anchors[1].position[1] - (double) anchors[0].position[1] , 2) + pow( (double) anchors[1].position[0] - (double) anchors[0].position[0] , 2) )/1000 , base_seperation_covar,
     (double) anchors[0].position[0]/1000,  (double) anchors[0].position[1]/1000, (double) anchors[1].position[0]/1000,  (double)anchors[1].position[1]/1000 );
    }

    //handle no val
    if(covariance_vec.size()<36){
      return;
    }
    boost::array<double, 36> covariance;
    for(int i = 0; i<36; i++){
      covariance[i]=covariance_vec[i];
    }

    ros::Time now = ros::Time::now();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp=now;
    pose_msg.header.seq=seq;
    seq++;
      pose_msg.header.frame_id="decawave";

    //take x and y off the back and put them in the pose
    pose_msg.pose.pose.position.y=covariance_vec[37];
    covariance_vec.pop_back();
    pose_msg.pose.pose.position.x=covariance_vec[36];
    covariance_vec.pop_back();
    pose_msg.pose.pose.position.z=0.0;
    pose_msg.pose.covariance=covariance;
    //quaternion, we dont know, fill w 0
    pose_msg.pose.pose.orientation.x=0.0;
    pose_msg.pose.pose.orientation.y=0.0;
    pose_msg.pose.pose.orientation.z=0.0;
    pose_msg.pose.pose.orientation.w=0.0;

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

//ASSUMES x,y is inside bounds. assumes p1 y is greater than p2 y
*/
std::vector<double> do_Math(double d0, double d0_err,double d1, double d1_err,double b,double b_err, double p1x, double p1y, double p2x, double p2y){
  //protect triangle inequality
  if ( abs(d0-d1) > b){
    std::cout << "vec: " << d0 << " " << d1 << " " << b << "\n";
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
  std::cout << "vec: " << d0 << " " << d1 << " " << b << "\n";
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
