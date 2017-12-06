#ifndef VISION_VIEWER_H
#define VISION_VIEWER_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud);


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer ("Matrix cloud");

// Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_col_handler (cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_col_handler, "cloud");

    viewer.addCoordinateSystem((0.0f,0.0f,0.0f));
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
    }
}

#endif  // VISION_VIEWER_H
