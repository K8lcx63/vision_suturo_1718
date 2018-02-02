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

void savePointCloudRGBNamed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            std::string filename);

void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void savePointCloudXYZNamed(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename);


void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud);

void savePointCloudPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud);

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals);



#endif  // VISION_SAVING_H
