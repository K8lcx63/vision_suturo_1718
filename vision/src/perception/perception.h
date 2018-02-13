//
// Created by tammo on 18.01.18.
//

#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "short_types.h"
#include "transformer/CloudTransformer.h"
#include "../saving/saving.h"
#include <pcl/features/cvfh.h>
#include <iterator>
#include <vector>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <eigen_conversions/eigen_msg.h>


std::vector<PointCloudRGBPtr>           findCluster(const PointCloudRGBPtr kinect);
PointStamped                            findCenterGazebo();
std::vector<geometry_msgs::PoseStamped> findPoses(const std::vector<PointCloudRGBPtr> clouds_in, std::vector<PointCloudVFHS308Ptr> vfhs_vector);
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
PointCloudVFHS308Ptr             cvfhRecognition(PointCloudRGBPtr input);
PointCloudRGBPtr                SACInitialAlignment(std::vector<PointCloudRGBPtr> objects,
                                                    std::vector<PointCloudVFHS308Ptr> features,
                                                    PointCloudRGBPtr target);
PointCloudRGBPtr                iterativeClosestPoint(PointCloudRGBPtr input, PointCloudRGBPtr target);
std::vector<uint8_t>              produceColorHist(PointCloudRGBPtr cloud);

extern PointCloudRGBPtr cloud_plane,
        cloud_cluster,
        cloud_cluster2,
        cloud_f,
        cloud_3df,
        cloud_voxelgridf,
        cloud_mlsf,
        cloud_prism,
        cloud_final;

#endif //VISION_PERCEPTION_H
