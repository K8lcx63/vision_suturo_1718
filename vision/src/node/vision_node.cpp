//
// Created by tammo on 21.01.18.
//

#include "vision_node.h"

CloudContainer cloudContainer;

const char *SIM_KINECT_POINTS_FRAME = "/head_mount_kinect/depth_registered/points";
const char *REAL_KINECT_POINTS_FRAME = "/kinect_head/depth_registered/points";

gazebo_msgs::GetModelState getmodelstate;

PointCloudXYZPtr scene(new pcl::PointCloud<pcl::PointXYZ>);

// ros::NodeHandle n_global;

std::vector<sensor_msgs::PointCloud2> objects_global;

std::string error_message; // Wird durch den Object Position Service mit ausgegeben

geometry_msgs::PointStamped centroid_stamped;


ros::Publisher pub_visualization;


// Use a callback function for the kinect subscriber to pass the NodeHandle to use in perception.h
void sub_kinect_callback(PointCloudXYZPtr kinect) {
    ROS_INFO("CALLBACK FUNCTION!");
    scene = kinect;
}

void start_node(int argc, char **argv) {
    ros::init(argc, argv, "suturo_vision");

/** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;
//n_global = n;
// Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(REAL_KINECT_POINTS_FRAME, 100, &sub_kinect_callback);

/** services and clients **/
// ServiceClient for calling the object position through gazebo
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee";  // Name des Objekts in Gazebo.

// Service for returning the object centroid
    ros::ServiceServer point_service = n.advertiseService("suturo_vision/objectPoint", getObjectPosition);
    ROS_INFO("%sPOINT SERVICE READY\n", "\x1B[32m");

// Service for returning if the object has fallen over already
    ros::ServiceServer pose_service = n.advertiseService("suturo_vision/objectPose", getObjectPose);
    ROS_INFO("%sPOSE SERVICE READY\n", "\x1B[32m");

    ros::ServiceServer object_service = n.advertiseService("suturo_vision/objectClusters", getObjects);
    ROS_INFO("%CLUSTERS SERVICE READY\n", "\x1B[32m");
// Visualization Publisher for debugging purposes
    ros::Publisher pub_visualization = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    centroid_stamped.point.x = 0, centroid_stamped.point.y = 0, centroid_stamped.point.z = 0;  // dummy point

    ros::Rate r(2.0);

    while (n.ok()) {
// ros::Publisher pub_objects =
// n.advertise<sensor_msgs::PointCloud2>("/vision/objects",1000);
// pub_objects.publish(objects_global);
// client.call(getmodelstate); // continously call model state


        pub_visualization.publish(publishVisualizationMarker(centroid_stamped)); // Update point for debug visualization

        ros::spinOnce();
        r.sleep();
    }

}

bool getObjectPose(vision_msgs::GetObjectClouds::Request &req,
                   vision_msgs::GetObjectClouds::Response &res) {
    // TODO
    return true;

}

bool getObjects(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res) {

    res.clouds.object_clouds = findCluster(scene);
    return true;

}

bool getObjectPosition(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res) {

    // TODO
    return true;
}