/*
 * decawavesim.cpp
 * Reports a 
 * VERSION: 0.0
 * Last changed: 2020-03-11
 * Authors: Julia Schatz <schat127@umn.edu>
 * Maintainers: Julia Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2020 UMN Robotics
 */

#include <iostream>

#include "decawave/decawave.h"
#include <tf/transform_listener.h>


namespace decawave{
DecawaveSim::DecawaveSim(std::string map_frame, std::string my_frame, int anchor0_id, int anchor1_id, std::string anchor0_frame, std::string anchor1_frame, std::string gazebo_prefix, std::string robot_frame_id, tf::TransformListener * tf) {
    this->my_frame = my_frame;
    this->map_frame = map_frame;
    this->anchor0_frame = anchor0_frame;
    this->anchor1_frame = anchor1_frame;
    this->tf = tf;
    this->gazebo_prefix = gazebo_prefix;
    this->robot_frame_id = robot_frame_id;
    this->anchor0_id = anchor0_id;
    this->anchor1_id = anchor1_id; 
}

std::vector<Anchor> DecawaveSim::updateSamples() {
    ros::Time now = ros::Time::now();

    tf::StampedTransform anchor0_trs;
    tf::StampedTransform anchor1_trs;
    if (!this->has_pos || 
        !tf->waitForTransform(this->map_frame, this->anchor0_frame, now, ros::Duration(0.5)) || 
        !tf->waitForTransform(this->map_frame, this->anchor1_frame, now, ros::Duration(0.5)) ||
        !tf->waitForTransform(this->robot_frame_id, this->my_frame, now, ros::Duration(0.5))) {
        ROS_INFO("Couldn't find transforms");
        return {}; // Can't find transforms
    }
    this->tf->lookupTransform(this->map_frame, this->anchor0_frame, now, anchor0_trs);
    this->tf->lookupTransform(this->map_frame, this->anchor1_frame, now, anchor1_trs);
    
    
    // Lookup position of base link
    int i = 0;
    for (; i < this->lastMsg.name.size(); i++) {
        if (this->lastMsg.name[i] == this->gazebo_prefix + "::" + this->robot_frame_id) {
            break;
        }
    }
    tf::StampedTransform tag_trs;
    this->tf->lookupTransform(this->robot_frame_id, this->my_frame, now, tag_trs);

    tf::Vector3 tagPos(this->lastMsg.pose[i].position.x + tag_trs.getOrigin().getX(), 
                       this->lastMsg.pose[i].position.y + tag_trs.getOrigin().getY(), 
                       this->lastMsg.pose[i].position.z + tag_trs.getOrigin().getZ());
    double range0 = (tagPos - anchor0_trs.getOrigin()).length() + this->gaussian(this->generator);
    double range1 = (tagPos - anchor1_trs.getOrigin()).length() + this->gaussian(this->generator);
    Anchor anchor0;
    Anchor anchor1;
    anchor0.id = this->anchor0_id;
    anchor0.distance = range0;
    anchor1.id = this->anchor1_id;
    anchor1.distance = range1;
    std::vector<Anchor> anchors = {anchor0, anchor1};
    return anchors;
}

void DecawaveSim::gazebo_joint_callback(const gazebo_msgs::LinkStates& msg) {
    this->lastMsg = msg;
    this->has_pos = true;
}

}
