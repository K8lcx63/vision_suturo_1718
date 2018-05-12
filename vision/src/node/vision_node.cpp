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

classifier my_classifier;

ros::Publisher pub_visualization;




// Use a callback function for the kinect subscriber to pass the NodeHandle to use in perception.h
/**
 * Callback-function saves the PointCloud received through the kinect
 * @param kinect PointCloud
 */
void sub_kinect_callback(sensor_msgs::PointCloud2 kinect) {
    pcl::fromROSMsg(kinect, *scene);
    cloud_perceived = scene;
    if (scene->size() == 0) {
        ROS_ERROR("Kinect has no image");
        error_message += "No image from kinect. ";
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_global.pose.position.x, pose_global.pose.position.y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, pose_global.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "object/pose"));

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
    ros::Subscriber sub = n.subscribe("object/pose", 10, &sub_kinect_callback);

    /** services and clients **/
    ros::ServiceServer object_service = n.advertiseService("vision_suturo/objects_information", getObjects);
    ros::ServiceServer pose_service = n.advertiseService("vision_suturo/objects_poses", getPoses);
    ROS_INFO("%sSuturo-Vision: Services ready\n", "\x1B[32m");

    // Visualization Publisher for debugging purposes
    ros::Publisher pub_visualization_object = n.advertise<sensor_msgs::PointCloud2>("vision_suturo/visualization_cloud",
                                                                                    0);


    ros::Publisher pub_perceived_object = n.advertise<sensor_msgs::PointCloud2>("vision_suturo/perceived_object", 0);
    ros::Publisher pub_mesh_object = n.advertise<sensor_msgs::PointCloud2>("vision_suturo/mesh_object", 0);

    ros::Publisher pub_aligned_object = n.advertise<sensor_msgs::PointCloud2>("vision_suturo/aligned_object", 0);

    ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("vision_suturo/pose", 0);
    

    ros::Rate r(2.0);

    std::string train_directory = "../../common_suturo1718/pcd_files";
    my_classifier.train(train_directory, false);

    ROS_INFO("%sVision is ready!\n", "\x1B[32m");


    while (n.ok()) {
        sensor_msgs::PointCloud2 cloud_final_pub;
        //ROS_INFO("%lu points", cloud_global->points.size());

        pcl::toROSMsg(*cloud_global, cloud_final_pub);
        cloud_final_pub.header.frame_id = "head_mount_kinect_rgb_optical_frame";
        pub_visualization_object.publish(cloud_final_pub);


        sensor_msgs::PointCloud2 cloud_perceived_pub;
        //ROS_INFO("%lu points", cloud_perceived->points.size());

        pcl::toROSMsg(*cloud_perceived, cloud_perceived_pub);
        cloud_perceived_pub.header.frame_id = "head_mount_kinect_rgb_optical_frame";
        pub_perceived_object.publish(cloud_perceived_pub);

        sensor_msgs::PointCloud2 cloud_mesh_pub;
        //ROS_INFO("%lu points", cloud_aligned->points.size());

        pcl::toROSMsg(*cloud_mesh, cloud_mesh_pub);
        cloud_mesh_pub.header.frame_id = "head_mount_kinect_rgb_optical_frame";
        pub_mesh_object.publish(cloud_mesh_pub);

        sensor_msgs::PointCloud2 cloud_aligned_pub;
        //ROS_INFO("%lu points", cloud_aligned->points.size());

        pcl::toROSMsg(*cloud_aligned, cloud_aligned_pub);
        cloud_aligned_pub.header.frame_id = "head_mount_kinect_rgb_optical_frame";
        pub_aligned_object.publish(cloud_aligned_pub);

        pub_pose.publish(pose_global);


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
    if (scene->points.size() < 500) {
        ROS_ERROR("Input from kinect is empty");
        error_message = "Cloud empty. ";
        // res.clouds.object_error = error_message;
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
    std::vector<std::string> classifier_results;
    for (int a = 0; a < all_clusters.size(); a++) { // Get histograms of each object and classify
        single_cvfh_features.clear();
        single_color_features.clear();
        for (int b = 0; b < 24; b++) {
            single_color_features.push_back(color_features_vector[b + (a * 24)]);
        }
        for (int c = 0; c < 308; c++) {
            single_cvfh_features.push_back(current_features_vector[c + (a * 308)]);
        }

        classifier_results.push_back(my_classifier.classify(single_color_features, single_cvfh_features));
    }

    res.clouds.labels = classifier_results;
    res.clouds.object_amount = all_clusters.size();
    //res.clouds.object_errors = error_message;

    return true;

}

bool getPoses(vision_suturo_msgs::poses::Request &req, vision_suturo_msgs::poses::Response &res) {
    // Get poses for the objects
    // Currently computes all centroids, but only takes the relevant one.

    if (!all_clusters.empty()) { // If objects have been perceived

        geometry_msgs::PoseStamped pose = findPose(all_clusters[req.index], req.labels);
        res.object_pose = pose;
    } else {
        geometry_msgs::PoseStamped dummy_pose;
        res.object_pose = dummy_pose;
        ROS_WARN("Returned empty pose. Call 'vision_suturo/objects_information' first!");
    }

    return true;
}
