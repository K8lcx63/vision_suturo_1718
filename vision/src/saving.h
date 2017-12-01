#ifndef VISION_SAVING_H
#define VISION_SAVING_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <pcl/impl/point_types.hpp>
#include "saving.h"

/**globale variablen **/

unsigned int filenr = 0;

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals);

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals) {
    ROS_INFO("SAVING FILES");
    std::stringstream ss;
    std::stringstream ss_input;
    std::stringstream ss_normals;

    // automatic save to $HOME/.ros folder

    ss << "./object_" << filenr << ".pcd";
    ss_input << "./kinect_" << filenr << ".pcd";
    ss_normals << "./kinect_normals_" << filenr << ".pcd";

    pcl::io::savePCDFileASCII(ss.str(), *objects);
    pcl::io::savePCDFileASCII(ss_input.str(), *kinect);
    pcl::io::savePCDFileASCII(ss_normals.str(), *normals);

    filenr++;
}
#endif  // VISION_SAVING_H
