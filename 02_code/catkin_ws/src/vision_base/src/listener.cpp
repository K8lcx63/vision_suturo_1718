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
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>


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
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(kinect_cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> *dummy_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_cloud, *dummy_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(dummy_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  // Get the plane model, if present.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setOptimizeCoefficients(true);
  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  if (planeIndices->indices.size() == 0)
  	std::cout << "Could not find a plane in the scene." << std::endl;
  else
  {
  	// Copy the points of the plane to a new cloud.
  	pcl::ExtractIndices<pcl::PointXYZ> extract;
  	extract.setInputCloud(cloud);
  	extract.setIndices(planeIndices);
  	extract.filter(*plane);

  	// Retrieve the convex hull.
  	pcl::ConvexHull<pcl::PointXYZ> hull;
  	hull.setInputCloud(plane);
  	// Make sure that the resulting hull is bidimensional.
  	hull.setDimension(2);
  	hull.reconstruct(*convexHull);

  	// Redundant check.
  	if (hull.getDimension() == 2)
  	{
  		// Prism object.
  		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
  		prism.setInputCloud(cloud);
  		prism.setInputPlanarHull(convexHull);
  		// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
  		// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
  		prism.setHeightLimits(0.0f, 0.5f);
  		pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

  		prism.segment(*objectIndices);

  		// Get and show all points retrieved by the hull.
  		extract.setIndices(objectIndices);
  		extract.filter(*objects);
  		pcl::visualization::CloudViewer viewerObjects("Objects on table");
  		viewerObjects.showCloud(objects);

  		while (!viewerObjects.wasStopped())
  		{
  			// Do nothing but wait.
  		}
  	}
  	else std::cout << "The chosen hull is not planar." << std::endl;
  	}
    // plane_out for advertising
    plane_out = objects;

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
	ros::Subscriber sub = n.subscribe("/head_mount_kinect/depth_registered/points", 1, &callback);
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
		pcl::toROSMsg(*plane_out, plane_out_msg);
		pub_plane_model.publish(plane_out_msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
