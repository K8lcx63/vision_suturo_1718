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

    ROS_INFO("STARTING TRAINING...");
    std::string xd = "../../common_suturo1718/pcd_files";
    train(xd, false);
    ROS_INFO("TRAINING FINISHED!");

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

    // If PR2 is not looking at anything.
    // This causes the whole segmentation and filtering process to be skipped if the cloud is empty
    // or too small to work on.
    if (scene->points.size() < 500)
    {
        ROS_ERROR("Input from kinect is empty");
        error_message = "Cloud empty. ";
        res.clouds.object_errors = error_message;
        return true;
    }
    // Execute findCluster()
    all_clusters = findCluster(scene);
    ROS_INFO("Suturo Vision: findCluster completed!");

    // Calculate features and put them into the message response
    std::vector<float> current_features_vector = getCVFHFeatures(all_clusters);
    std::vector<uint64_t> color_features_vector = getColorFeatures(all_clusters);

    std::vector<float> single_cvfh_features;
    std::vector<uint64_t> single_color_features;
    for(int a = 0; a < all_clusters.size(); a++){ // Get histograms of each object and classify
        for(int b = 0; b < 24; b++){
            single_color_features.push_back(color_features_vector[b+(a*24)]);
        }
        for(int c = 0; c < 308; c++){
            single_cvfh_features.push_back(current_features_vector[c+(a*308)]);
        }
        classify(single_color_features, single_cvfh_features);
    }
    //getAllFeatures(all_clusters, current_features_vector, color_features_vector);

    // estimate poses (quaternions)



    res.clouds.normal_features = current_features_vector;
    res.clouds.color_features = color_features_vector;
    res.clouds.object_amount = all_clusters.size();
    res.clouds.object_errors = error_message;

    return true;

}

bool getPoses(vision_suturo_msgs::poses::Request &req, vision_suturo_msgs::poses::Response &res) {
    // Get poses for the objects
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
