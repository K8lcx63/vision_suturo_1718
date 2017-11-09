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

//

//

//

//

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
void findCluster(const sensor_msgs::PointCloud2 cloud_msg) {
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(cloud_msg, pcl_pc);

  pcl::PointCloud<pcl::PointXYZ> *oldcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc, *oldcloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(oldcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::SACSegmentation<pcl::PointXYZ> seg; // Das (RAN)SAC Segmentations-Objekt
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter writer;
  // Parameter für das Segmentations-Objekt
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int i = 0, nr_points = (int)cloud->points.size();
  while (cloud->points.size() > 0.3 * nr_points) { // Während noch mindestens 30% der originalen Punktwolke da sind... (diesen Wert anpassen?)
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() < min_plane_inliers) {
      std::cerr << "No estimation possible. Please adjust min_plane_inliers or "
                   "dataset!"
                << std::endl;
      break;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    extract.filter(*cloud_plane);

    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud = *cloud_f;
    // *cluster_out = *cloud_f;

  }

	// --- END same as findSupportPlane --- //

  // Creating the KdTree object for searching the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cluster(
      new pcl::PointCloud<pcl::PointXYZ>);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;
    j++;
    *result_cluster = *cloud_cluster;
  }
  *cluster_out = *result_cluster;


}


//

//

//

//

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

void findSupportPlane(const sensor_msgs::PointCloud2 cloud_msg)
{
	// Sensor_msgs to PCL_Pointcloud
 pcl::PCLPointCloud2 pcl_pc;
 pcl_conversions::toPCL(cloud_msg, pcl_pc);

 // PCL Pointcloud to Pointcloud<pcl::PointXYZ> for segmentation
 pcl::PointCloud<pcl::PointXYZ> *oldcloud(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromPCLPointCloud2(pcl_pc, *oldcloud);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(oldcloud);

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(
		 new pcl::PointCloud<pcl::PointXYZ>),
		 cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

 pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
 pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
 // Create the segmentation object
 pcl::SACSegmentation<pcl::PointXYZ> seg;
 // Optional
 seg.setOptimizeCoefficients(true);
 // Set ModelType, Algorithm, Iterations and distances between Points and
 // neighbors
 seg.setModelType(pcl::SACMODEL_PLANE);
 seg.setMethodType(pcl::SAC_RANSAC);
 seg.setMaxIterations(1000);
 seg.setDistanceThreshold(0.01);

 // Extract and filter plane
 pcl::ExtractIndices<pcl::PointXYZ> extract;

 int i = 0, nr_points = (int)cloud->points.size();
 // While 30% of the original cloud is still there
 while (cloud->points.size() > 0.3 * nr_points) {
	 // Segment the largest planar component from the remaining cloud
	 seg.setInputCloud(cloud);
	 seg.segment(*inliers, *coefficients);

	 // Check for minimum inliers required for estimation
	 if (inliers->indices.size() < min_plane_inliers) {
		 std::cerr << "No estimation possible. Please adjust min_plane_inliers or "
									"dataset!"
							 << std::endl;
		 break;
	 }

	 // Extract planar component
	 extract.setInputCloud(cloud);
	 extract.setIndices(inliers);
	 extract.setNegative(false);
	 extract.filter(*cloud_p);
	 std::cerr << "Planar component: " << cloud_p->width * cloud_p->height
						 << " data points." << std::endl;

	 // Create the filtering object
	 extract.setNegative(true);
	 extract.filter(*cloud_f);
	 cloud.swap(cloud_f);
	 i++;
 }
 *plane_out = *cloud_f;
}

int main(int argc, char **argv)
{
	// Subscriber für das points-Topic des Kinect-Sensors.
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, callback);
  ros::Subscriber subPlane = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, &findSupportPlane);
  ros::Subscriber subCluster = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, &findCluster);

	// ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

	// Service für das Übergeben der Eistee-Position aus Gazebo.
	ros::ServiceServer service = n.advertiseService("VisObjectInfo", getObjectInfo);
	ROS_INFO("Service steht bereit.");



	// DEBUG! Hier kann der Service abgerufen werden, oben aber nicht?
  sensor_msgs::PointCloud2 cloud_conv;
  ros::Rate r(20.0);
	while (n.ok()){
		client.call(getmodelstate);

    // plane_cloud Publisher
   sensor_msgs::PointCloud2 plane_cloud;
   ros::Publisher plane_pub = n.advertise<sensor_msgs::PointCloud2>("plane_cloud_pubby", 1000);
   pcl::toROSMsg(*plane_out, plane_cloud);
   plane_pub.publish(plane_cloud);

   //cluster_cloud Publisher
   sensor_msgs::PointCloud2 cluster_cloud;
   ros::Publisher cluster_pub = n.advertise<sensor_msgs::PointCloud2>("cluster_cloud_pubby", 1000);
   pcl::toROSMsg(*cluster_out, cluster_cloud);
   cluster_pub.publish(cluster_cloud);

   ros::spinOnce();
   r.sleep();
	}

	return 0;
}
