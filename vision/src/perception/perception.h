//
// Created by tammo on 18.01.18.
//

#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "short_types.h"
#include "transformer/CloudTransformer.h"
#include "../saving/saving.h"

#include <iterator>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>


std::vector<PointCloudRGBPtr>           findCluster(const PointCloudRGBPtr kinect);
PointStamped                            findCenterGazebo();
geometry_msgs::PoseStamped      findPose(const PointCloudRGBPtr input, std::string label);
PointCloudNormalPtr             estimateSurfaceNormals(PointCloudRGBPtr input);
PointCloudPointNormalPtr        createPointNormals(PointCloudRGBPtr input,
                                                   PointCloudNormalPtr normals);
PointIndices                    estimatePlaneIndices(PointCloudRGBPtr input);
PointCloudRGBPtr                extractCluster(PointCloudRGBPtr input,
                                               PointIndices indices,
                                               bool negative);
PointCloudRGBPtr                apply3DFilter(PointCloudRGBPtr input,
                                              float x,
                                              float y,
                                              float z);
PointCloudRGBPtr                mlsFilter(PointCloudRGBPtr input);
std::vector<PointCloudRGBPtr>   euclideanClusterExtraction(PointCloudRGBPtr input);
PointCloudRGBPtr                voxelGridFilter(PointCloudRGBPtr input);
PointCloudRGBPtr                outlierRemoval(PointCloudRGBPtr input);
PointCloudVFHS308Ptr            cvfhRecognition(PointCloudRGBPtr input);
PointCloudRGBPtr                SACInitialAlignment(PointCloudRGBPtr input, PointCloudRGBPtr target);
PointCloudRGBPtr                iterativeClosestPoint(PointCloudRGBPtr input, PointCloudRGBPtr target);
std::vector<uint64_t>           produceColorHist(PointCloudRGBPtr cloud);
void                            getAllFeatures(std::vector<PointCloudRGBPtr> all_clusters,
                                               std::vector<float> vfhs_vector,
                                               std::vector<uint64_t> color_features_vector);

std::vector<float>              getCVFHFeatures(std::vector<PointCloudRGBPtr> all_clusters);
std::vector<uint64_t>           getColorFeatures(std::vector<PointCloudRGBPtr> all_clusters);
PointCloudRGBPtr                getTargetByLabel(std::string label);


extern PointCloudRGBPtr cloud_global;

#endif //VISION_PERCEPTION_H
