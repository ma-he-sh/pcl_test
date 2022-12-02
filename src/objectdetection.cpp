//
// Created by mahesh on 11/19/22.
//

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf_conversions/tf_eigen.h"
#include <Eigen/Core>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

ros::Publisher pub;
ros::Publisher pub3;
ros::Publisher pub4;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_data) {
    std::string lidar_frame_id;
    sensor_msgs::Image image_;

    // cloud data not dense
    if(( cloud_data->width * cloud_data->height ) == 0 ) return;

    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 downsampleCloud;    // only object
    pcl::PCLPointCloud2 downsampleCloud3;   // segmented objects
    pcl::PCLPointCloud2 downsampleCloud4;   // objects flattened

    // create pcl clouddata
    pcl_conversions::toPCL(*cloud_data, *cloud);

    float leafSize;
    ros::param::get("/leaf_size", leafSize);
    float minDepth;
    //ros::param::get("/min_depth", minDepth);
    float maxDepth;
    //ros::param::get("/max_depth", maxDepth);
    float compensate;
    ros::param::get("/compensate", compensate);

    bool debug;
    ros::param::get("/debug", debug);

    try {
        ros::param::get("/max_distance", maxDepth);
        maxDepth = maxDepth - compensate;
    } catch( ros::Exception &e ) {
        std::cout << "Exception Occured::" << e.what() << "\n";
        ros::param::get("/max_depth", maxDepth);
    }

    try {
        ros::param::get("/min_distance", minDepth);
    } catch( ros::Exception &e ) {
        std::cout << "Exception Occured::" << e.what() << "\n";
        ros::param::get("/min_depth", minDepth);
    }

    if (debug) {
        std::cout << maxDepth << "\n";
        std::cout << minDepth << "\n";
    }
    

    // filter cloud
    pcl::VoxelGrid <pcl::PCLPointCloud2> voxelCloud;
    voxelCloud.setInputCloud(cloudPtr);
    voxelCloud.setLeafSize(leafSize, leafSize, leafSize);
    voxelCloud.filter(downsampleCloud);

    downsampleCloud3 = downsampleCloud;
    downsampleCloud4 = downsampleCloud;


    // background and object
    pcl::PointCloud <pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(downsampleCloud, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
    tree->setInputCloud(point_cloudPtr);

    // clustering the pointclouds
    std::vector <pcl::PointIndices> segments;
    pcl::EuclideanClusterExtraction <pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(0.01);
    ec.setMaxClusterSize(6000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloudPtr);
    ec.extract(segments);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_objects(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_segmented_objects(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_objects_flattened(new pcl::PointCloud <pcl::PointXYZRGB>);

    // cluster handling
    int k = 0;
    int colorIndex = 10;
    for (std::vector<pcl::PointIndices>::const_iterator it = segments.begin();it != segments.end(); ++it) {
        for (std::vector<int>::const_iterator p_it = it->indices.begin(); p_it != it->indices.end(); ++p_it) {
            pcl::PointXYZRGB point;
            pcl::PointXYZRGB point3;
            pcl::PointXYZRGB point4;

            // create new points
            point.x = point_cloudPtr->points[*p_it].x;
            point.y = point_cloudPtr->points[*p_it].y;
            point.z = point_cloudPtr->points[*p_it].z;

            point3.x = point_cloudPtr->points[*p_it].x;
            point3.y = point_cloudPtr->points[*p_it].y;
            point3.z = point_cloudPtr->points[*p_it].z;

            point4.x = point_cloudPtr->points[*p_it].x;
            point4.y = point_cloudPtr->points[*p_it].y;
            point4.z = 0;

            // in observable space
            if( point.z >= minDepth && point.z <= maxDepth ) {
                point.r = colorIndex;
                point.g = 0;
                point.b = 255;

                point3.r = 255;
                point3.g = 255 * k / (segments.size());
                point3.b = 50;

                point4.r = colorIndex;
                point4.g = 0;
                point4.b = 255;

                all_objects_flattened->push_back(point4);
            } else {
                point.r = 0;
                point.g = 0;
                point.b = 0;

                point3.r = 0;
                point3.g = 0;
                point3.b = 0;
            }

            all_objects->push_back(point);
            all_segmented_objects->push_back(point3);
        }

        k++;
        colorIndex++;
    }

    // messages
    sensor_msgs::PointCloud2 voxel; // objects 
    sensor_msgs::PointCloud2 voxel3; // segmented objects
    sensor_msgs::PointCloud2 voxel4; // only objects


    all_objects->header.frame_id = point_cloudPtr->header.frame_id;
    if (all_objects->size()) {
        pcl::toPCLPointCloud2(*all_objects, downsampleCloud);
    } else {
        pcl::toPCLPointCloud2(*point_cloudPtr, downsampleCloud);
    }

    // convert pcl to pointcloud
    pcl_conversions::fromPCL(downsampleCloud, voxel);

    all_segmented_objects->header.frame_id = point_cloudPtr->header.frame_id;
    if (all_segmented_objects->size()) {
        pcl::toPCLPointCloud2(*all_segmented_objects, downsampleCloud3);
    } else {
        pcl::toPCLPointCloud2(*point_cloudPtr, downsampleCloud3);
    }

    // convert pcl to pointcloud
    pcl_conversions::fromPCL(downsampleCloud3, voxel3);

    all_objects_flattened->header.frame_id = point_cloudPtr->header.frame_id;
    if (all_objects_flattened->size()) {
        pcl::toPCLPointCloud2(*all_objects_flattened, downsampleCloud4);
    } else {
        pcl::toPCLPointCloud2(*point_cloudPtr, downsampleCloud4);
    }

    // convert pcl to pointcloud
    pcl_conversions::fromPCL(downsampleCloud4, voxel4);


    // publish to topic
    pub.publish(voxel);
    pub3.publish(voxel3);
    pub4.publish(voxel4);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_test");
    ros::NodeHandle nh;

    // subscribe to camera top
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloud_cb);

    // create voxel topics
    pub = nh.advertise<sensor_msgs::PointCloud2>("/voxel/objects", 1);
    pub3 = nh.advertise<sensor_msgs::PointCloud2>("/voxel/segmented_objects", 1);
    pub4 = nh.advertise<sensor_msgs::PointCloud2>("/voxel/only_objects", 1);

    ros::spin();
}
