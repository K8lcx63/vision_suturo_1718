#ifndef VISION_SAVING_H
#define VISION_SAVING_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <pcl/impl/point_types.hpp>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <iostream>

#include "saving.h"

std::stringstream getTime();
void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud);
void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals);



std::stringstream getTime(){
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::stringstream time_string = std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    return time_string;
}
void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ROS_INFO("Saving PointCloud<PointXYZ>");
    std::stringstream ss;

    // automatic save to $HOME/.ros folder

    ss << "./cloudXYZ_" << getTime()  << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *cloud);

}

void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud) {
    ROS_INFO("Saving PointCloud<Normal>");
    std::stringstream ss;

    // automatic save to $HOME/.ros folder

    ss << "./cloudNormal_" << getTime()  << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *cloud);

}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals) {
    ROS_INFO("SAVING FILES");
    std::stringstream ss;
    std::stringstream ss_input;
    std::stringstream ss_normals;

    // automatic save to $HOME/.ros folder

    ss << "./object_" << getTime()  << ".pcd";
    ss_input << "./kinect_" << getTime()  << ".pcd";
    ss_normals << "./kinect_normals_" << getTime()  << ".pcd";

    pcl::io::savePCDFileASCII(ss.str(), *objects);
    pcl::io::savePCDFileASCII(ss_input.str(), *kinect);
    pcl::io::savePCDFileASCII(ss_normals.str(), *normals);

}
#endif  // VISION_SAVING_H
