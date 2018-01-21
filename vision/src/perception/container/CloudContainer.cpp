//
// Created by tammo on 21.01.18.
//

#include "CloudContainer.h"

CloudContainer::CloudContainer() {

    void setInputCloud(PointCloudXYZPtr input);


    void setObjectClouds(std::vector<sensor_msgs::PointCloud2> object_clouds);



};

void CloudContainer::setInputCloud(PointCloudXYZPtr input){
    kinect = input;
}
void CloudContainer::setObjectClouds(std::vector<sensor_msgs::PointCloud2> object_clouds){
    objects = object_clouds;
}
