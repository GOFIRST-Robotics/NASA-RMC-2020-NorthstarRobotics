#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <cmath>

ros::Publisher pub;
static float _min_percentage = 5;
static float _max_distance = 5;
static float _max_vert_angle = 0.087;

double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr &msg){

    // Convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud_msg);

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
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_max_distance);

    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZ>);
    int original_size(cloud->height*cloud->width);
    int n_planes(0);
    while (cloud->height*cloud->width>original_size*_min_percentage/100){

        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Make sure we could find a plane
        if (inliers->indices.size() == 0) {
            break;
        }
        
        // Determine whether this is a vertical plane
        // Normalized dot product with (0,0,1)
        float* vals = coefficients->values;
        float cos_angle = vals[2] / sqrt(vals[0]*vals[0] + vals[1]*vals[1] + vals[2] * vals[2]);
        if (cos_angle > cos(_max_vert_angle)) {
            // This plane is not vertical, we can pass it through
            for (int i=0;i<inliers->indices.size();i++){

                // Get Point
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

                // Copy point to noew cloud
                cloud_pub->points.push_back(pt);
            }
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
    pub.publish(cloud_publish);
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "pc_filter_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("pointcloud_input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_filtered", 1);

  // Spin
  ros::spin ();
}