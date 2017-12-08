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

std::string getTime();

void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud);

void savePointCloudPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals);



std::string getTime() {
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[120];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%d-%m-%Y-%I-%M-%S", timeinfo);
    std::string str(buffer);


    return str;
}

void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ROS_INFO("Saving PointCloud<PointXYZ>");
    std::string time_string = getTime();

    std::stringstream ss;

    // automatic save to $HOME/.ros folder

    ss << "./cloudXYZ_" << time_string << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *cloud);

}

void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud) {
    ROS_INFO("Saving PointCloud<Normal>");
    std::string time_string = getTime();

    std::stringstream ss;

    // automatic save to $HOME/.ros folder

    ss << "./cloudNormal_" << time_string << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *cloud);

}

void savePointCloudPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    ROS_INFO("Saving PointCloud<PointNormal>");
    std::string time_string = getTime();

    std::stringstream ss;

    // automatic save to $HOME/.ros folder

    ss << "./cloudPointNormal_" << time_string << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *cloud);

}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals) {
    ROS_INFO("SAVING FILES");
    std::string time_string = getTime();
    std::stringstream ss;
    std::stringstream ss_input;
    std::stringstream ss_normals;

    // automatic save to $HOME/.ros folder

    ss << "./object_" << time_string << ".pcd";
    ss_input << "./kinect_" << time_string << ".pcd";
    ss_normals << "./kinect_normals_" << time_string << ".pcd";

    pcl::io::savePCDFileASCII(ss.str(), *objects);
    pcl::io::savePCDFileASCII(ss_input.str(), *kinect);
    pcl::io::savePCDFileASCII(ss_normals.str(), *normals);

}


#endif  // VISION_SAVING_H
