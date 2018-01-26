//
// Created by tammo on 21.01.18.
//

#ifndef VISION_VISION_NODE_H
#define VISION_VISION_NODE_H
#include <vision_msgs/GetObjectClouds.h>
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "vision_msgs/Histogram.h"
#include "vision_msgs/GetObjectInfo.h"
#include "vision_msgs/ObjectClouds.h"
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include "../perception/container/CloudContainer.h"
#include "../viewer/viewer.h"
#include "../perception/perception.h"

#include "../perception/short_types.h"

bool getObjectPosition(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);
bool getObjectPose(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);
bool getObjects(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);
void sub_kinect_callback(PointCloudXYZPtr kinect);
void start_node(int argc, char **argv);

#endif //VISION_VISION_NODE_H
