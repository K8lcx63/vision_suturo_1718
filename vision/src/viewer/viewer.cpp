//
// Created by tammo on 18.01.18.
//

#include "viewer.h"
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


visualization_msgs::Marker publishVisualizationMarker(geometry_msgs::PointStamped point) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = point.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "vision";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}

