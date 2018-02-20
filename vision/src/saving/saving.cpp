//
// Created by tammo on 18.01.18.
//
#include "saving.h"

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

void savePointCloudRGBNamed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename) {
    try {
        ROS_INFO("Saving PointCloud<PointXYZRGB>");
        std::string time_string = getTime();

        std::stringstream ss;

        // automatic save to $HOME/.ros folder

        ss << "/home/tammo/pcd/" << filename << "_" << time_string << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}

void savePointCloudXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    try {
        ROS_INFO("Saving PointCloud<PointXYZ>");
        std::string time_string = getTime();

        std::stringstream ss;

        // automatic save to $HOME/.ros folder

        ss << "../../../src/vision_suturo_1718/vision/data/cloudXYZ_" << time_string << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}


void savePointCloudXYZNamed(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename) {
    try {
        ROS_INFO("Saving PointCloud<PointXYZ>");
        std::string time_string = getTime();

        std::stringstream ss;

        // automatic save to $HOME/.ros folder

        ss << "../../../src/vision_suturo_1718/vision/data/" << filename << "_" << time_string << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}

void savePointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud) {
    try {
        ROS_INFO("Saving PointCloud<Normal>");
        std::string time_string = getTime();

        std::stringstream ss;

        // automatic save to $HOME/.ros folder

        ss << "../../../src/vision_suturo_1718/vision/data/cloudNormal_" << time_string << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}

void savePointCloudPointNormal(pcl::PointCloud<pcl::PointNormal>::Ptr cloud) {
    try {
        ROS_INFO("Saving PointCloud<PointNormal>");
        std::string time_string = getTime();

        std::stringstream ss;

        // automatic save to $HOME/.ros folder

        ss << "../../../src/vision_suturo_1718/vision/data/cloudPointNormal_" << time_string << ".pcd";
        pcl::io::savePCDFileASCII(ss.str(), *cloud);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                    pcl::PointCloud<pcl::Normal>::Ptr normals) {
    try {
        ROS_INFO("SAVING FILES");
        std::string time_string = getTime();
        std::stringstream ss;
        std::stringstream ss_input;
        std::stringstream ss_normals;

        // automatic save to $HOME/.ros folder

        ss << "../../../src/vision_suturo_1718/vision/data/object_" << time_string << ".pcd";
        ss_input << "../../../src/vision_suturo_1718/vision/data/kinect_" << time_string << ".pcd";
        ss_normals << "../../../src/vision_suturo_1718/vision/data/kinect_normals_" << time_string << ".pcd";

        pcl::io::savePCDFileASCII(ss.str(), *objects);
        pcl::io::savePCDFileASCII(ss_input.str(), *kinect);
        pcl::io::savePCDFileASCII(ss_normals.str(), *normals);
    } catch (pcl::PCLException e) {
        ROS_ERROR("Saving failed: %s", e.what());
    }

}
