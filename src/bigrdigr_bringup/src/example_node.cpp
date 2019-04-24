/*
 * example_node.cpp // // Note, global FILE_NAME is "example_node"
 * INSERT_DESC_HERE
 * VERSION: VERSION_NO
 * Last changed: 2018-09-24
 * Authors: My Name <myname@umn.edu>
 * Maintainers: Goldy <goldy@umn.edu>
 * MIT License
 * Copyright (c) 2018 GOFIRST-Robotics
 */

// ROS Libs
#include <ros/ros.h>
#include "AUTOGEN_DEPS"
// #include <pub-sub_nameN_type.h>

//:BEGIN Native_Libs
#include <string>
// User inputs #include <iostream>, etc
//:END

//:BEGIN Custom_Libs
// User inputs custom libs
// #include <opencv/core.h>
// #include <pcl/pcl.h>
// #include <my_lib/lib.h>
// extern "C" {
//   #include "my_c_lib/clib.h"
// }
//:END

// Subscribers (inputs)
//    sub_name1 (sub_name1_type): sub_name1_TOPIC_VALUE
//      sub_name1_desc
//    sub_name2 (sub_name2_type): sub_name2_TOPIC_VALUE
//      sub_name2_desc
//    sub_name3 (sub_name3_type): sub_name3_TOPIC_VALUE
//      sub_name3_desc

// Publishers (outputs)
//    pub_name1 (pub_name1_type): pub_name1_TOPIC_VALUE
//      pub_name1_desc
//    pub_name2 (pub_name2_type): pub_name2_TOPIC_VALUE
//      pub_name2_desc
//    pub_name3 (pub_name3_type): pub_name3_TOPIC_VALUE
//      pub_name3_desc

// Parameters (settings)
//    param_name1 (param_name1_type): default=param_name1_default(,param_name1_path)
//    param_name2 (param_name2_type): default=param_name2_default(,param_name1_path)
//    param_name3 (param_name3_type): default=param_name3_default(,param_name1_path)


// ROS Node and Publishers
ros::NodeHandle * nh;
ros::Publisher pub_name1_pub;
ros::Publisher pub_name2_pub;
ros::Publisher pub_name3_pub;

// ROS Topics
std::string sub_name1_topic = sub_name1_TOPIC_VALUE;
std::string sub_name2_topic = sub_name2_TOPIC_VALUE;
std::string sub_name3_topic = sub_name3_TOPIC_VALUE;
std::string pub_name1_topic = pub_name1_TOPIC_VALUE;
std::string pub_name2_topic = pub_name2_TOPIC_VALUE;
std::string pub_name3_topic = pub_name3_TOPIC_VALUE;

// ROS Callbacks
void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg);
void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg);
void sub_name3_callback(const sub_name3_typeLHS::sub_name3_typeRHS::ConstPtr& msg);

// ROS Params
// // For normal types: string, int, float, bool
param_name1_type param_name1 = param_name1_default;
param_name2_type param_name2 = param_name2_default;
param_name3_type param_name3 = param_name3_default;
// // For "lists", aka "arrays", given option of map or vector
std::map<param_name1_type_BASE, param_name_1_type_BASE> param_name_1 = param_name1_default;
std::vector<param_name1_type_BASE> param_name1 = param_name2_default;

//:BEGIN Global_Vars
// double global1 = 0.0;
// int    global2 = 0;
// std::string global3 = "";
// void myCustomFunc1(int i);
// bool myCustomFunc2(float f);
//:END

int main(int argc, char** argv){
  // Init ROS
  ros::init(argc, argv, FILE_NAME);
  nh = new ros::NodeHandle("~");

  // Subscribers
  ros::Subscriber sub_name1_sub = nh->subscribe(sub_name1_topic, sub_name1_BUFLEN, sub_name1_callback);
  ros::Subscriber sub_name2_sub = nh->subscribe(sub_name2_topic, sub_name2_BUFLEN, sub_name2_callback);
  ros::Subscriber sub_name3_sub = nh->subscribe(sub_name3_topic, sub_name3_BUFLEN, sub_name3_callback);

  // Publishers
  pub_name1_pub = nh->advertise<pub_name1_typeLHS::pub_name1_typeRHS>(pub_name1_topic, pub_name1_BUFLEN);
  pub_name2_pub = nh->advertise<pub_name2_typeLHS::pub_name2_typeRHS>(pub_name2_topic, pub_name2_BUFLEN);
  pub_name3_pub = nh->advertise<pub_name3_typeLHS::pub_name3_typeRHS>(pub_name3_topic, pub_name3_BUFLEN);

  // Params
  nh->param<param_name1_type>(param_name1_path, param_name1, param_name1_default;
  nh->param<param_name2_type>(param_name2_path, param_name2, param_name2_default;
  nh->param<param_name3_type>(param_name3_path, param_name3, param_name3_default;

  // Spin
  ros::spin();
}


void sub_name1_callback(const sub_name1_typeLHS::sub_name1_typeRHS::ConstPtr& msg){
//:BEGIN sub_name1_callback

//:END
}

void sub_name2_callback(const sub_name2_typeLHS::sub_name2_typeRHS::ConstPtr& msg){
//:BEGIN sub_name2_callback

//:END
}

void sub_name3_callback(const sub_name3_typeLHS::sub_name3_typeRHS::ConstPtr& msg){
//:BEGIN sub_name3_callback

//:END
}
