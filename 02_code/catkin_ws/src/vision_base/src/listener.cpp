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
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

gazebo_msgs::GetModelState getmodelstate;
ros::ServiceClient client;

pcl::PointCloud<pcl::PointXYZ>::Ptr objects_output (new pcl::PointCloud<pcl::PointXYZ>);

// pcl::PointCloud<pcl::PointXYZ>::Ptr computeCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)

void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);
void findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);
void findCenterNKN(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

bool getObjectInfo(object_detection::VisObjectInfo::Request &req, object_detection::VisObjectInfo::Response &res);

int main(int argc, char **argv)
{
	// Subscriber für das points-Topic des Kinect-Sensors.
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub_kinect = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, &findCluster);

	// ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

	// Service für das Übergeben der Eistee-Position aus Gazebo.
	ros::ServiceServer service = n.advertiseService("VisObjectInfo", getObjectInfo);
	ROS_INFO("Service steht bereit.");



	// DEBUG! Hier kann der Service abgerufen werden, oben aber nicht?

	ros::Rate r(10.0);
	while (n.ok()){
		client.call(getmodelstate);
		ros::Publisher pub_plane_model = n.advertise<sensor_msgs::PointCloud2>("vision_base/objects",1000);
		sensor_msgs::PointCloud2 objects_output_msg;
		pcl::toROSMsg(*objects_output, objects_output_msg);
		pub_plane_model.publish(objects_output_msg);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_y(new pcl::PointCloud<pcl::PointXYZ>), cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    // writer for saving pcd-files
    pcl::PCDWriter writer;
    std::stringstream ss;

    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (kinect);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.5, 0.5);
    pass.setKeepOrganized(false);
    pass.filter (*cloud_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud (cloud_x);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.0, 1.0);
    pass.setKeepOrganized(false);
    pass.filter (*cloud_y);

    // Create the filtering object (z-axis)
    pass.setInputCloud (cloud_y);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    pass.setKeepOrganized(false);
    pass.filter (*cloud);

    /** Get the plane model, if present **/
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

        /** copy objects to new cloud **/
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(planeIndices);
        extract.setNegative(true);
        extract.filter(*objects);

        pcl::visualization::CloudViewer viewerObjects("Objects on table");
        findCenter(objects);
        viewerObjects.showCloud(objects);
        while (!viewerObjects.wasStopped()) {
            // Do nothing but wait.
        }
    }
}

void findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)
{

	int cloud_size = object_cloud->points.size();

	float sum_x = 0, sum_y = 0, sum_z = 0;

	for(int i = 0; i < cloud_size; i++)
	{
		pcl::PointXYZ point = object_cloud->points[i];

		sum_x += point.x;
		sum_y += point.y;
		sum_z += point.z;
	}

	geometry_msgs::Point centroid;
	centroid.x = sum_x/cloud_size;
	centroid.y = sum_y/cloud_size;
	centroid.z = sum_z/cloud_size;

	std::cout << "Zentrum des Eistee-Objekts: " << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;



    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (object_cloud);

    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    pcl::PointXYZ searchPoint;

    searchPoint.x = centroid.x;
    searchPoint.y = centroid.y;
    searchPoint.z = centroid.z;

    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    sum_x = 0, sum_y = 0, sum_z = 0;

    for(int i = 0; i < pointIdxNKNSearch.size(); i++)
    {
        pcl::PointXYZ point = object_cloud->points[ pointIdxNKNSearch[i] ];

        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }

    geometry_msgs::Point centroidNKN;
    centroidNKN.x = sum_x/cloud_size;
    centroidNKN.y = sum_y/cloud_size;
    centroidNKN.z = sum_z/cloud_size;

    std::cout << "Zentrum des Eistee-Objekts (KNN): " << centroidNKN.x << " " << centroidNKN.y << " " << centroidNKN.z << std::endl;

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

void findCenterNKN(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)
{

    int cloud_size = object_cloud->points.size();

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for(int i = 0; i < cloud_size; i++)
    {
        pcl::PointXYZ point = object_cloud->points[i];

        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }

    geometry_msgs::Point centroid;
    centroid.x = sum_x/cloud_size;
    centroid.y = sum_y/cloud_size;
    centroid.z = sum_z/cloud_size;

    std::cout << "Zentrum des Eistee-Objekts: " << centroid.x << " " << centroid.y << " " << centroid.z << std::endl;



    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (object_cloud);

    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    pcl::PointXYZ searchPoint;

    searchPoint.x = centroid.x;
    searchPoint.y = centroid.y;
    searchPoint.z = centroid.z;

    kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    sum_x = 0, sum_y = 0, sum_z = 0;

    for(int i = 0; i < pointIdxNKNSearch.size(); i++)
    {
        pcl::PointXYZ point = object_cloud->points[ pointIdxNKNSearch[i] ];

        sum_x += point.x;
        sum_y += point.y;
        sum_z += point.z;
    }

    geometry_msgs::Point centroidNKN;
    centroidNKN.x = sum_x/cloud_size;
    centroidNKN.y = sum_y/cloud_size;
    centroidNKN.z = sum_z/cloud_size;

    std::cout << "Zentrum des Eistee-Objekts (KNN): " << centroidNKN.x << " " << centroidNKN.y << " " << centroidNKN.z << std::endl;

}