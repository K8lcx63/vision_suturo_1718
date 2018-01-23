//
// Created by tammo on 18.01.18.
//

#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "short_types.h"
#include "transformer/CloudTransformer.h"
#include <pcl/features/cvfh.h>


std::vector<sensor_msgs::PointCloud2> findCluster(const PointCloudRGBPtr kinect);

geometry_msgs::PointStamped findCenterGazebo();

std::vector<geometry_msgs::PointStamped> findCenter(const std::vector<sensor_msgs::PointCloud2> object_cloud);

PointCloudNormalPtr estimateSurfaceNormals(PointCloudRGBPtr input);

PointCloudPointNormalPtr
createPointNormals(PointCloudRGBPtr input, PointCloudNormalPtr normals);

PointIndices estimatePlaneIndices(PointCloudRGBPtr input);

PointCloudRGBPtr extractCluster(PointCloudRGBPtr input, PointIndices indices, bool negative);

PointIndices prismSegmentation(PointCloudRGBPtr input_cloud, PointCloudRGBPtr plane);

PointCloudRGBPtr apply3DFilter(PointCloudRGBPtr input, float x, float y,
                               float z);

PointCloudRGBPtr mlsFilter(PointCloudRGBPtr input);

PointCloudRGBPtr voxelGridFilter(PointCloudRGBPtr input);

PointCloudRGBPtr outlierRemoval(PointCloudRGBPtr input);

pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhRecognition(PointCloudXYZPtr input);

#endif //VISION_PERCEPTION_H
