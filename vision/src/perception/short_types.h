//
// Created by tammo on 21.01.18.
//

#ifndef VISION_SHORT_TYPES_H
#define VISION_SHORT_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudPointNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointIndices::Ptr PointIndices;
typedef std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointCloudXYZPtrVector;
typedef std::vector<pcl::PointIndices> PointIndicesVector;

#endif //VISION_SHORT_TYPES_H
