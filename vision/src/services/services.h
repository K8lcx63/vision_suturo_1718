//
// Created by tammo on 21.01.18.
//

#ifndef VISION_SERVICES_H
#define VISION_SERVICES_H


#include <vision_msgs/GetObjectClouds.h>

bool getObjectPosition(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);

bool getObjectPose(vision_msgs::GetObjectClouds::Request &req,
                   vision_msgs::GetObjectClouds::Response &res);

bool getObjects(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);
#endif //VISION_SERVICES_H
