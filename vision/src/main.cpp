#include "actionlib/server/simple_action_server.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/PointCloud2.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"

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

unsigned int input_noise_threshold = 12;


/** function heads **/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

geometry_msgs::Point
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

bool getObjectInfo(object_detection::VisObjectInfo::Request &req,
                   object_detection::VisObjectInfo::Response &res);

/** main function **/
int main(int argc, char **argv) {
    // Subscriber für das points-Topic des Kinect-Sensors.
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub_kinect = n.subscribe(
            "/head_mount_kinect/depth_registered/points", 100, &findCluster);

    // ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
    ros::ServiceClient client =
            n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

    // Service für das Übergeben der Eistee-Position aus Gazebo.
    ros::ServiceServer service =
            n.advertiseService("VisObjectInfo", getObjectInfo);
    ROS_INFO("%sPOINT SERVICE READY\n", "\x1B[32m");

    filenr = 0; // apply numbers for saving pcd files
    kinect_point.x = 0, kinect_point.y = 0, kinect_point.z = 0; // dummy point
    ros::Rate r(2.0);

    // Parameter testing stuff...
    bool simulation;


    while (n.ok()) {
        client.call(getmodelstate);

        // Parameter testing stuff... MAYBE rather automatically check if getmodelstate is valid after starting the node
        if (n.getParam("/simulation", simulation))
        {
            ROS_INFO("Parameter simulation exists!");
            if(simulation)
            {
                ROS_INFO("Parameter simulation is true!");
            }
            else
            {
                ROS_INFO("Parameter simulation is false!");
            }
        }
        else
        {
            ROS_WARN("Parameter simulation does not exist!");
        }


        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

/**
 ** Finde den Eistee!
**/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_y(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);


    if (kinect->points.size() < input_noise_threshold){ // if PR2 is not looking at anything
        ROS_ERROR("INPUT CLOUD EMPTY (PR2: \"OH, MY GOD! I AM BLIND!\"");


    } else {
        ROS_INFO("CLUSTER EXTRACTION STARTED");
        // Objects for storing the point clouds.


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

        if (cloud->points.size() == 0){
            ROS_ERROR("NO CLOUD AFTER FILTERING");
        } else {
            // ROS_INFO("FINDING PLANE");
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZ> segmentation;
            segmentation.setInputCloud(cloud);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(0.01); // Distance to model points
            segmentation.setOptimizeCoefficients(true);
            segmentation.segment(*planeIndices, *coefficients);


            if (planeIndices->indices.size() == 0)
                ROS_ERROR("NO PLANE FOUND");
            else {

                // ROS_INFO("EXTRACT CLUSTER");
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud);
                extract.setIndices(planeIndices);
                extract.setNegative(true);
                extract.filter(*objects);

                if (objects->points.size() == 0) {
                    ROS_ERROR("EXTRACTED CLUSTER IS EMPTY");
                } else{

                    kinect_point = findCenter(objects);

                    // clouds for saving
                    kinect_global = kinect;
                    objects_global = objects;

                }

            }
        }
    }






}

geometry_msgs::Point
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud) {

    if (object_cloud->points.size() != 0){
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

        ROS_INFO("%sCURRENT CLUSTER CENTER\n", "\x1B[32m");
        ROS_INFO("\x1B[32mX: %f\n", centroid.y);
        ROS_INFO("\x1B[32mY: %f\n", centroid.x);
        ROS_INFO("\x1B[32mZ: %f\n", centroid.z);

        return centroid;
    } else {
        ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
    }

}

bool getObjectInfo(object_detection::VisObjectInfo::Request &req,
                   object_detection::VisObjectInfo::Response &res) {
    ROS_INFO("POINT SERVICE CALLED");
    geometry_msgs::PointStamped kinect_point_stamped;
    kinect_point_stamped.point = kinect_point;
    // Eventuell doch ...kinect_rgb_optical...
    kinect_point_stamped.header.frame_id = "/head_mount_kinect_ir_optical_frame";
    res.object.position = kinect_point_stamped;
    if (kinect_point.x == 0 && kinect_point.y == 0 && kinect_point.z == 0) {
        ROS_ERROR("No point found. Is the input point cloud empty?");
        res.object.error = "No point found. Is the input point cloud empty?";
    }
    else
    {
        res.object.error = "";
    }
    //ROS_INFO("GOT EXTRACTED POINT");
    // when service is called, input cloud (kinect) and output cloud (extracted
    // objects) from findCluster are saved to ./data
    // savePointCloud(objects_global, kinect_global);
    return true;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect) {

        ROS_INFO("SAVING FILES");
        std::stringstream ss;
        std::stringstream ss_input;

        ss << getenv("HOME") << "/catkin_ws/src/vision_suturo_1718/vision/data/object_" << filenr << ".pcd";
        ss_input << getenv("HOME") << "/catkin_ws/src/vision_suturo_1718/vision/data/kinect_" << filenr << ".pcd";

        pcl::io::savePCDFileASCII(ss.str(), *objects);
        pcl::io::savePCDFileASCII(ss_input.str(), *kinect);

        filenr++;



}