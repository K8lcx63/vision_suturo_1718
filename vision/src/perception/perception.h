//
// Created by tammo on 18.01.18.
//

#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H

#include <Eigen/Geometry>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
// #include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
/** for icp and alignment **/
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPointNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointIndices::Ptr PointIndices;



class CloudContainer {


    PointCloudXYZPtr kinect;
    std::vector<sensor_msgs::PointCloud2> objects;

public:

    CloudContainer();

    void setInputCloud(PointCloudXYZPtr input);

    void setObjectClouds(std::vector<sensor_msgs::PointCloud2> object_clouds);

    const PointCloudXYZPtr &getKinect() const {
        return kinect;
    }

    const std::vector<sensor_msgs::PointCloud2> &getObjects() const {
        return objects;
    }

    virtual ~CloudContainer() {

    }
};/** Function Headers **/

std::vector<sensor_msgs::PointCloud2> findCluster(const PointCloudXYZPtr kinect, ros::NodeHandle n);

geometry_msgs::PointStamped findCenterGazebo();

std::vector<geometry_msgs::PointStamped> findCenter(const std::vector<sensor_msgs::PointCloud2> object_cloud);

PointCloudNormalPtr estimateSurfaceNormals(PointCloudXYZPtr input);

PointCloudPointNormalPtr
createPointNormals(PointCloudXYZPtr input, PointCloudNormalPtr normals);

PointIndices estimatePlaneIndices(PointCloudXYZPtr input);

PointCloudXYZPtr extractCluster(PointCloudXYZPtr input, PointIndices indices, bool negative);

PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane);

PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z);

PointCloudXYZPtr mlsFilter(PointCloudXYZPtr input);

PointCloudXYZPtr voxelGridFilter(PointCloudXYZPtr input);

PointCloudXYZPtr outlierRemoval(PointCloudXYZPtr input );

#endif //VISION_PERCEPTION_H
