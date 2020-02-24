/*
 * pc_filter_node.cpp
 * Filters vertical planes out of a pointcloud
 * VERSION: 1.0.0
 * Last changed: 2020-02-24
 * Authors: Julia Schatz <schat127@umn.edu>
 * Maintainers: Julia Schatz <schat127@umn.edu>
 * MIT License
 * Copyright (c) 2020 UMN Robotics
 */

/* Interface: 
 *  Pub: 
 *   filter_pub (sensor_msgs/PointCloud2): "pointcloud_filtered"
 *  Sub:
 *   sub (sensor_msgs/PointCloud2): "pointcloud_input"
 * Param: 
 *   min_percentage (float) 5; The cutoff percentage of point cloud left to stop finding new planes
 *   distance_threshold (float) 5; The RANSAC distance threshold, in inches
 *   max_vert_angle (float) 0.087; The maximum angle away from vertical that constitutes a vertical plane, in radians
 *   map_frame (string) map; The frame to transform point clouds into 
 */

// ROS libs
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

// Native libs
#include <cmath>
#include <string>

// ROS Nodes and publishers
ros::Publisher filter_pub;
ros::NodeHandle * nh;
ros::NodeHandle * pnh;

// ROS Params
static float _min_percentage = 5;
static float _dist_threshold = 5;
static float _max_vert_angle = 0.087;
static std::string _map_frame = "map";

// ROS callbacks
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg);

// Global variables
tf::TransformListener tflistener;

// Utility methods
double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients);

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "pc_filter_node");
  nh = new ros::NodeHandle();
  pnh = new ros::NodeHandle("~");

  // Params
  pnh->getParam("min_percentage", _min_percentage);
  pnh->getParam("distance_threshold", _dist_threshold);
  pnh->getParam("max_vert_angle", _max_vert_angle);
  pnh->getParam("map_frame", _map_frame);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh->subscribe ("pointcloud_input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  filter_pub = nh->advertise<sensor_msgs::PointCloud2> ("pointcloud_filtered", 1);

  // Spin
  ros::spin();
}

double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients) {
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud_msg);
    // Transform into map frame so we can check for real vertical
    pcl_ros::transformPointCloud(_map_frame, *cloud_msg, *cloud_msg, tflistener);

    // Filter cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_msg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits(0.001,10000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud);

    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::PointXYZ> passer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_dist_threshold);

    passer.setInputCloud(cloud);
    passer.setNegative(false);

    float original_size = cloud->height*cloud->width;
    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZ>);
    while (cloud->height * cloud->width > original_size * _min_percentage / 100){

        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Make sure we could find a plane
        if (inliers->indices.size() == 0) {
            break;
        }
        
        // Determine whether this is a vertical plane
        // Normalized dot product with (0,0,1)
        std::vector<float> vals = coefficients->values;
        float cos_angle = vals[2] / sqrt(vals[0]*vals[0] + vals[1]*vals[1] + vals[2] * vals[2]);
        if (cos_angle > cos(_max_vert_angle)) {
            // This plane is not vertical, we can pass it through
            pcl::PointXYZ pt = cloud->points[inliers->indices[i]];
            passer.setIndices(inliers);
            extract.filter(cloud_pub);
        }

        // Remove plane points from original cloud
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);

        // Next iteration
        n_planes++;
    }

    // Publish points
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_pub,cloud_publish);
    cloud_publish.header = msg->header;
    cloud_publish.header.frame = _map_frame;
    filter_pub.publish(cloud_publish);
}