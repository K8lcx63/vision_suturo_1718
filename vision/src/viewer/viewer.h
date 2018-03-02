#ifndef VISION_VIEWER_H
#define VISION_VIEWER_H

#include <iostream>

#include <geometry_msgs/PointStamped.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

void visualizeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

// Visualization publisher stuff
visualization_msgs::Marker publishVisualizationMarker(geometry_msgs::PointStamped point);

#endif  // VISION_VIEWER_H
