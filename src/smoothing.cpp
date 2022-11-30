//
// Created by mahesh on 11/23/22.
//
//
// Created by mahesh on 11/19/22.
//

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_data) {
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);


    // Publish the model coefficients
    pub.publish (cloud_data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_test");
    ros::NodeHandle nh;

    // subscribe to camera top
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloud_cb);

    // create voxel topic
    pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel_smooth", 1);

    ros::spin();
}
