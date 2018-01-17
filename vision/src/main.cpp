#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "vision_msgs/GetObjectInfo.h"
#include "vision_msgs/GetObjectClouds.h"
#include "visualization_msgs/Marker.h"
// includes for pcl

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "globals.h"
#include "perception.h"

bool getObjectPosition(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res);

bool getObjectPose(vision_msgs::GetObjectClouds::Request &req,
                   vision_msgs::GetObjectClouds::Response &res);

// Visualization publisher stuff
visualization_msgs::Marker publishVisualizationMarker(geometry_msgs::PointStamped point);

ros::Publisher pub_visualization;


// Use a callback function for the kinect subscriber to pass the NodeHandle to use in perception.h
void sub_kinect_callback(PointCloudXYZPtr kinect) {
    ROS_INFO("CALLBACK FUNCTION!");
    kinect_global = kinect; // save perceived Pointclouds
}

/** main function **/
int main(int argc, char **argv) {
    // Subscriber für das points-Topic des Kinect-Sensors.
    ros::init(argc, argv, "vision_main");

    /** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;

    // Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(REAL_KINECT_POINTS_FRAME, 100, &sub_kinect_callback);

    /** services and clients **/
    // ServiceClient for calling the object position through gazebo
    ros::ServiceClient client =
            n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee";  // Name des Objekts in Gazebo.

    // Service for returning the object centroid
    ros::ServiceServer point_service =
            n.advertiseService("vision_main/objectPoint", getObjectPosition); //VisObjectInfo
    ROS_INFO("%sPOINT SERVICE READY\n", "\x1B[32m");

    // Service for returning if the object has fallen over already
    ros::ServiceServer pose_service =
            n.advertiseService("vision_main/objectPose", getObjectPose);
    ROS_INFO("%sPOSE SERVICE READY\n", "\x1B[32m");

    // Visualization Publisher for debugging purposes
    ros::Publisher pub_visualization = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    centroid_stamped.point.x = 0, centroid_stamped.point.y = 0,
    centroid_stamped.point.z = 0;  // dummy point
    ros::Rate r(2.0);


    while (n.ok()) {
        // ros::Publisher pub_objects =
        // n.advertise<sensor_msgs::PointCloud2>("/vision_main/objects",1000);
        // pub_objects.publish(objects_global);
        // client.call(getmodelstate); // continously call model state


        bool simulation =
                client.exists();  // Periodically check if this is a simulation

        ROS_INFO("XD");

        pub_visualization.publish(publishVisualizationMarker(centroid_stamped)); // Update point for debug visualization

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

bool getObjectPosition(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res) {
    ROS_INFO("POINT SERVICE CALLED");

    std::vector<geometry_msgs::PointStamped> centroids = findCenter(objects_global);

    for (auto cent : centroids) {
        cent.header.stamp = ros::Time::now();  // Timestamp
    }


    res.clouds.object_clouds_centroids = centroids;

    for (auto schtring : res.clouds.object_clouds_information) {
        schtring = error_message;  // Let planning know if any problems occured
    }
    return true;
}

bool getObjectPose(vision_msgs::GetObjectClouds::Request &req,
                   vision_msgs::GetObjectClouds::Response &res) {
    /**
    res.clouds.object_clouds_poses
     */
    return true;

}

bool getObjects(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res) {

    res.clouds.object_clouds = findCluster(kinect_global, n_global);
    if (res.clouds.object_clouds_centroids.size() == 0) {
        ROS_INFO("Remember to call point service!");
    }
    res.clouds.object_clouds_information[0] = "super toller dummy";
    return true;

}

visualization_msgs::Marker publishVisualizationMarker(geometry_msgs::PointStamped point) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = point.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "vision";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.7;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}