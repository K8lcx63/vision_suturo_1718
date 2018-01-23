//
// Created by tammo on 21.01.18.
//

#include "CloudContainer.h"

CloudContainer::CloudContainer() {

    void setInputCloud(PointCloudXYZPtr input);

    // std::vector<sensor_msgs::PointCloud2>
    void setObjectClouds(sensor_msgs::PointCloud2 object_clouds);



};

void CloudContainer::setInputCloud(PointCloudXYZPtr input){
    kinect = input;
}
    // std::vector<sensor_msgs::PointCloud2>
void CloudContainer::setObjectClouds(sensor_msgs::PointCloud2 object_clouds){
    objects = object_clouds;
}
