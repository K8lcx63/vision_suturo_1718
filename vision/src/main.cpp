#include "actionlib/server/simple_action_server.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/PointCloud2.h"
#include "vision_msgs/ObjectDetection.h"
#include "vision_msgs/VisObjectInfo.h"

// includes for pcl

#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

/** global variables **/
gazebo_msgs::GetModelState getmodelstate;
geometry_msgs::Point kinect_point;
pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_global;
pcl::PointCloud<pcl::PointXYZ>::Ptr objects_global;


unsigned int filenr;



/** function heads **/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

geometry_msgs::Point
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

bool getObjectInfo(vision_msgs::VisObjectInfo::Request &req,
                   vision_msgs::VisObjectInfo::Response &res);

/** main function **/
int main(int argc, char **argv) {
    // Subscriber für das points-Topic des Kinect-Sensors.
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub_kinect = n.subscribe(
            "/head_mount_kinect/depth_registered/points", 1000, &findCluster);

    // ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
    ros::ServiceClient client =
            n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

    // Service für das Übergeben der Eistee-Position aus Gazebo.
    ros::ServiceServer service =
            n.advertiseService("VisObjectInfo", getObjectInfo);
    ROS_INFO("Service steht bereit.");

    filenr = 0; // apply numbers for saving pcd files
    ros::Rate r(2.0);
    while (n.ok()) {
        client.call(getmodelstate);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

/**
 ** Finde den Eistee!
**/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect) {
    ROS_INFO("STARTING CLUSTER EXTRACTION");
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_y(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(
            new pcl::PointCloud<pcl::PointXYZ>);

    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(kinect);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.5, 0.5);
    pass.setKeepOrganized(false);
    pass.filter(*cloud_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud(cloud_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, 1.0);
    pass.setKeepOrganized(false);
    pass.filter(*cloud_y);

    // Create the filtering object (z-axis)
    pass.setInputCloud(cloud_y);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.8);
    pass.setKeepOrganized(false);
    pass.filter(*cloud);

    ROS_INFO("GETTING PLANE MODEL");
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    if (planeIndices->indices.size() == 0)
        std::cout << "Could not find a plane in the scene." << std::endl;
    else {

        ROS_INFO("EXTRACTING PLANE FROM CLOUD");
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(planeIndices);
        extract.setNegative(true);
        extract.filter(*objects);

        kinect_point = findCenter(objects);

        // clouds for saving
        kinect_global = kinect;
        objects_global = objects;

        ROS_INFO("CLUSTER EXTRACTED");
    }
}

geometry_msgs::Point
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud) {

    int cloud_size = object_cloud->points.size();

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < cloud_size; i++) {
        pcl::PointXYZ point = object_cloud->points[i];

        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }

    geometry_msgs::Point centroid;
    centroid.x = sum_x / cloud_size;
    centroid.y = sum_y / cloud_size;
    centroid.z = sum_z / cloud_size;

    ROS_INFO("CURRENT CLUSTER CENTER: ");
    ROS_INFO("X: %f", centroid.y);
    ROS_INFO("Y: %f", centroid.x);
    ROS_INFO("Z: %f", centroid.z);

    return centroid;
}

bool getObjectInfo(vision_msgs::VisObjectInfo::Request &req,
                   vision_msgs::VisObjectInfo::Response &res) {
    ROS_INFO("Der Service wurde abgerufen.");
    if (kinect_point.x == 0 && kinect_point.y == 0 && kinect_point.z == 0) {
        ROS_INFO("Kein Punkt aus den kinect-Daten verfügbar. Verwende stattdessen "
                         "Daten aus gazebo.");
        geometry_msgs::Point point; // Der Punkt, der später übergeben wird.
        point.x = getmodelstate.response.pose.position.x;
        point.y = getmodelstate.response.pose.position.y;
        point.z = getmodelstate.response.pose.position.z;

        res.object.position = point;
    } else {
        res.object.position = kinect_point;

        // when service is called, input cloud (kinect) and output cloud (extracted
        // objects) from findCluster are saved to ./data
        savePointCloud(objects_global, kinect_global);
    }
    res.object.type = "";

    // ROS_INFO("x=%ld, y=%ld, z=%ld", res.object.position.x,
    // res.object.position.y, res.object.position.z);

    return true;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect) {
    std::stringstream ss;
    std::stringstream ss_input;
    ss << "/home/tammo/catkin_ws/src/vision_suturo_1718/vision/data/object_"
       << filenr << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *objects);

    ss_input << "/home/tammo/catkin_ws/src/vision_suturo_1718/vision/data/kinect_"
             << filenr << ".pcd";
    pcl::io::savePCDFileASCII(ss_input.str(), *kinect);

    ROS_INFO("CLUSTER CLOUD SAVED TO FILE");

    filenr++;
}