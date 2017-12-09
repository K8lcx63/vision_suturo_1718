#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "vision_msgs/GetObjectInfo.h"

// includes for pcl

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "globals.h"
#include "perception.h"
#include "saving.h"
#include "viewer.h"

bool getObjectPosition(object_detection::VisObjectInfo::Request &req,
                       object_detection::VisObjectInfo::Response &res);

bool getObjectPose(vision_msgs::GetObjectInfo::Request &req,
                   vision_msgs::GetObjectInfo::Response &res);

/** main function **/
int main(int argc, char **argv) {
    // Subscriber für das points-Topic des Kinect-Sensors.
    ros::init(argc, argv, "vision_main");

    /** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;

    // Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(
            "/head_mount_kinect/depth_registered/points", 100, &findCluster);

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

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

bool getObjectPosition(object_detection::VisObjectInfo::Request &req,
                       object_detection::VisObjectInfo::Response &res) {
    ROS_INFO("POINT SERVICE CALLED");
    centroid_stamped.header.stamp = ros::Time::now();  // Timestamp
    res.object.position =
            centroid_stamped;  // Result of findCenter or findCenterGazebo
    res.object.error =
            error_message;  // Let planning know if any problems occured

    // when service is called, input cloud (kinect) and output cloud (extracted
    // objects) from findCluster are saved to ./data

    /** estimate object normals **/
    ROS_INFO("Estimating Normals for Reference");
    normals_global = estimateSurfaceNormals(objects_global);

    /** save clouds**/
    savePointCloud(objects_global, kinect_global, normals_global);





    return true;
}

bool getObjectPose(vision_msgs::GetObjectInfo::Request &req,
                   vision_msgs::GetObjectInfo::Response &res) {
    if (objectIsStanding() == 1) {

        res.info.isStanding = 1;
        res.info.information = "Objekt steht";
    } else if (objectIsStanding() == 0){
        res.info.isStanding = 0;
        res.info.information = "Objekt liegt";
    }
}
