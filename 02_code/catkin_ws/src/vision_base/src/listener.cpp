#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Point.h"
#include "actionlib/server/simple_action_server.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"

// includes for pcl

#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Includes for planar-segmentation

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Includes for cluster extraction
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


// #include <type_traits>

// Zum Verhindern von "was not declared in this scope"-Fehlern:
gazebo_msgs::GetModelState getmodelstate;
ros::ServiceClient client;

// required variables
int min_plane_inliers = 1000;
int min_cluster_size = 100;
float radius_search = 0.03; // for normals estimation

// Pointer to the most recent normals
pcl::PointCloud<pcl::Normal>::Ptr normals_out(new pcl::PointCloud<pcl::Normal>);

// Pointer to supporting plane
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out (new pcl::PointCloud<pcl::PointXYZ>);

// Pointer to object cluster object
pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_out (new pcl::PointCloud<pcl::PointXYZ>);

/**
 ** Find an object-cluster (i.e. Box of passierte Tomaten)
**/
void findCluster(const sensor_msgs::PointCloud2 kinect_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(kinect_cloud, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZ> *dummy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud, *dummy_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(dummy_cloud);

      std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;


      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }
        // Extract the inliers
            extract.setInputCloud(cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*plane_cloud);
            std::cerr << "PointCloud representing the planar component: " << plane_cloud->width * plane_cloud->height << " data points." << std::endl;

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);
            //i++;
            plane_out = plane_cloud;
          }



void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("I heard: something"); // Gib das aus, funktioniert nicht so richtig
}

bool getObjectInfo(object_detection::VisObjectInfo::Request &req, object_detection::VisObjectInfo::Response &res)
{
	ROS_INFO("Der Service wurde abgerufen.");
	geometry_msgs::Point point; // Der Punkt, der später übergeben wird.
	point.x = getmodelstate.response.pose.position.x;
	point.y = getmodelstate.response.pose.position.y;
	point.z = getmodelstate.response.pose.position.z;

	res.object.position = point;
	res.object.type = "";

	// ROS_INFO("x=%ld, y=%ld, z=%ld", res.object.position.x, res.object.position.y, res.object.position.z);

	return true;
}


int main(int argc, char **argv)
{
	// Subscriber für das points-Topic des Kinect-Sensors.
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/head_mount_kinect/depth_registered/points", 1, callback);
  ros::Subscriber sub_kinect = n.subscribe("/head_mount_kinect/depth_registered/points", 1, &findCluster);

	// ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

	// Service für das Übergeben der Eistee-Position aus Gazebo.
	ros::ServiceServer service = n.advertiseService("VisObjectInfo", getObjectInfo);
	ROS_INFO("Service steht bereit.");



	// DEBUG! Hier kann der Service abgerufen werden, oben aber nicht?

  ros::Rate r(20.0);
	while (n.ok()){
		client.call(getmodelstate);
    ros::Publisher pub_plane_model = n.advertise<sensor_msgs::PointCloud2>("plane_model",1);
    sensor_msgs::PointCloud2 plane_out_msg;
    pcl::toROSMsg(plane_out, plane_out_msg);
    pub_plane_model.publish(plane_out_msg);

   ros::spinOnce();
   r.sleep();
	}

	return 0;
}
