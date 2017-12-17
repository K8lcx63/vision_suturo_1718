#ifndef VISION_VIEWER_H
#define VISION_VIEWER_H

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

void visualizeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);


void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::visualization::PCLVisualizer viewer("Matrix cloud");

// Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_col_handler(cloud, 255, 255, 255);
    viewer.addPointCloud(cloud, cloud_col_handler, "cloud");

    viewer.addCoordinateSystem((0.0f, 0.0f, 0.0f));
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

void visualizeTwoPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
    pcl::visualization::PCLVisualizer viewer("Cloud 1: White . Cloud 2: Red");

// Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_col_handler(cloud1, 255, 255, 255);
    viewer.addPointCloud(cloud1, cloud1_col_handler, "cloud1");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_col_handler(cloud2, 255, 0, 0);
    viewer.addPointCloud(cloud2, cloud2_col_handler, "cloud2");


    viewer.addCoordinateSystem((0.0f, 0.0f, 0.0f));
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

void visualizeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);

    }
}


#endif  // VISION_VIEWER_H
