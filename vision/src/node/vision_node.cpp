//
// Created by tammo on 21.01.18.
//

#include "vision_node.h"

CloudContainer cloudContainer;

const char *SIM_KINECT_POINTS_FRAME = "/head_mount_kinect/depth_registered/points";
const char *REAL_KINECT_POINTS_FRAME = "/kinect_head/depth_registered/points";
const char *PCD_KINECT_POINTS_FRAME = "/cloud_pcd";

gazebo_msgs::GetModelState getmodelstate;

PointCloudRGBPtr scene(new PointCloudRGB);

// ros::NodeHandle n_global;


std::string error_message; // Wird durch den Object Position Service mit ausgegeben

geometry_msgs::PointStamped centroid_stamped;


ros::Publisher pub_visualization;


// Use a callback function for the kinect subscriber to pass the NodeHandle to use in perception.h
/**
 * Callback-function saving the pointcloud received by the Kinect-Camera
 * @param kinect pointcloud received from kinect
 */
void sub_kinect_callback(PointCloudRGBPtr kinect) {

    ROS_INFO("CALLBACK FUNCTION!");
    if (kinect->size() == 0){
        ROS_ERROR("Kinect has no image");
        error_message += "No image from kinect. ";
    }
    scene = kinect;
}

/**
 * starts the node for processing the pointclouds and communicating with other nodes
 * @param argc unused for now
 * @param argv unused for now
 */
void start_node(int argc, char **argv) {
    ros::init(argc, argv, "suturo_vision");

    /** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;

    // Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(REAL_KINECT_POINTS_FRAME, 10, &sub_kinect_callback);

    /** services and clients **/
    // ServiceClient for calling the object position through gazebo
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee";  // Name des Objekts in Gazebo.

    ros::ServiceServer object_service = n.advertiseService("suturo_vision/objectClusters", getObjects);
    ROS_INFO("%sCLUSTERS SERVICE READY\n", "\x1B[32m");

    // Visualization Publisher for debugging purposes
    ros::Publisher pub_visualization_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    ros::Publisher pub_visualization_object = n.advertise<sensor_msgs::PointCloud2>("visualization_cloud", 0);

    ros::Rate r(2.0);

    while (n.ok()) {
        pub_visualization_marker.publish(publishVisualizationMarker(centroid_stamped)); // Update point for debug visualization
        pub_visualization_object.publish(cloudContainer.getObjects());

        ros::spinOnce();
        r.sleep();
    }

}

/**
 * Service to extract objects from scene to work with and to get all required information from them
 * @param req empty request
 * @param res returns all members from ObjectClouds.msg
 * @return true if service call succeeded, false otherwise
 */
bool getObjects(vision_msgs::GetObjectClouds::Request &req, vision_msgs::GetObjectClouds::Response &res) {

    // Execute findCluster()
    std::vector<PointCloudRGBPtr> all_clusters = findCluster(scene);
    ROS_INFO("findCluster completed!");

    // Calculate features and put them into the message response
    float* current_features;
    std::vector<float> current_features_vector;
    for(int i = 0; i < all_clusters.size(); i++) {
        current_features = cvfhRecognition(all_clusters[i]);

        for(int x = 0; x < 308; x++){
            ROS_INFO("%f", current_features[x]);
            current_features_vector.push_back(current_features[x]);
        }
    }

    res.clouds.features = current_features_vector;
    std::cout << "cvfh filling completed" << std::endl;

    return true;

}
