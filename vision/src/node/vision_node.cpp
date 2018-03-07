//
// Created by tammo on 21.01.18.
//

#include "vision_node.h"
#include <tf/transform_broadcaster.h>
const char *SIM_KINECT_POINTS_FRAME = "/head_mount_kinect/depth_registered/points";
const char *REAL_KINECT_POINTS_FRAME = "/kinect_head/depth_registered/points";
const char *PCD_KINECT_POINTS_FRAME = "/cloud_pcd";


PointCloudRGBPtr scene(new PointCloudRGB);

// ros::NodeHandle n_global;


std::string error_message; // Wird durch den Object Position Service mit ausgegeben

geometry_msgs::PointStamped centroid_stamped;

std::vector<PointCloudRGBPtr> all_clusters;


ros::Publisher pub_visualization;




// Use a callback function for the kinect subscriber to pass the NodeHandle to use in perception.h
/**
 * Callback-function saves the PointCloud received through the kinect
 * @param kinect PointCloud
 */
void sub_kinect_callback(PointCloudRGBPtr kinect) {

    if (kinect->size() == 0) {
        ROS_ERROR("Kinect has no image");
        error_message += "No image from kinect. ";
    }
    scene = kinect;

}

/**
 * Starts the node for processing the PointClouds and communicating with other nodes
 * @param argc unused for now
 * @param argv unused for now
 */
void start_node(int argc, char **argv) {
    ros::init(argc, argv, "vision_suturo");

    /** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;

    // Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(REAL_KINECT_POINTS_FRAME, 10, &sub_kinect_callback);

    /** services and clients **/

    ros::ServiceServer object_service = n.advertiseService("vision_suturo/objects_information", getObjects);
    ros::ServiceServer pose_service = n.advertiseService("vision_suturo/objects_poses", getPoses);
    ROS_INFO("%sSuturo-Vision: Services ready\n", "\x1B[32m");

    // Visualization Publisher for debugging purposes
    ros::Publisher pub_visualization_object = n.advertise<sensor_msgs::PointCloud2>("vision_suturo/visualization_cloud", 0);

    ros::Rate r(2.0);

    while (n.ok()) {
        sensor_msgs::PointCloud2 cloud_final_pub;
        ROS_INFO("%lu points", cloud_global->points.size());

        pcl::toROSMsg(*cloud_global, cloud_final_pub);
        cloud_final_pub.header.frame_id = "head_mount_kinect_ir_optical_frame";
        pub_visualization_object.publish(cloud_final_pub);
        ros::spinOnce();
        r.sleep();
    }

}

/**
 * Service to extract objects from scene to work with and to get all required information from them.
 * @param req empty request
 * @param res returns all members from ObjectsInfo.msg
 * @return true if service call succeeded, false otherwise
 */
bool getObjects(vision_suturo_msgs::objects::Request &req, vision_suturo_msgs::objects::Response &res) {

    // Execute findCluster()
    //std::vector<PointCloudRGBPtr> all_clusters = findCluster(scene);
    all_clusters = findCluster(scene);
    ROS_INFO("Suturo Vision: findCluster completed!");

    // Calculate features and put them into the message response
    std::vector<float> current_features_vector = getCVFHFeatures(all_clusters);
    std::vector<uint64_t> color_features_vector = getColorFeatures(all_clusters);
    //getAllFeatures(all_clusters, current_features_vector, color_features_vector);

    // estimate poses (quaternions)



    res.clouds.normal_features = current_features_vector;
    res.clouds.color_features = color_features_vector;
    res.clouds.object_amount = all_clusters.size();

    return true;

}

bool getPoses(vision_suturo_msgs::poses::Request &req, vision_suturo_msgs::poses::Response &res) {
    // Get poses for the objects
    // TODO: Use object information of the last object_information service call!
    // TODO: Only return a single Pose, depending on the given index!
    // Currently computes all centroids, but only takes the relevant one.

    if(!all_clusters.empty()) { // If objects have been perceived

        geometry_msgs::PoseStamped pose = findPose(all_clusters[req.index], req.labels);
        res.object_pose = pose;
    }
    else{
        geometry_msgs::PoseStamped dummy_pose;
        res.object_pose = dummy_pose;
        ROS_WARN("Returned empty pose. Call 'vision_suturo/objects_information' first!");
    }

    return true;
}
