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


std::vector<sensor_msgs::PointCloud2> findCluster(const PointCloudXYZPtr kinect);

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

PointCloudXYZPtr outlierRemoval(PointCloudXYZPtr input);

#endif //VISION_PERCEPTION_H
