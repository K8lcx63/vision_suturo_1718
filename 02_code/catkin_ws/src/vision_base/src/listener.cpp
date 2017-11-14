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
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>



// #include <type_traits>

// Zum Verhindern von "was not declared in this scope"-Fehlern:
gazebo_msgs::GetModelState getmodelstate;
ros::ServiceClient client;

// Pointer to supporting plane
pcl::PointCloud<pcl::PointXYZ>::Ptr plane_out (new pcl::PointCloud<pcl::PointXYZ>);

// Pointer to object cluster object
pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_out (new pcl::PointCloud<pcl::PointXYZ>);

/**
 ** Finde den Eistee!
**/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::visualization::CloudViewer viewerObjects("Objects");

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_before(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_before2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_before3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;


    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f(0,0,1));
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(false);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);


        if (planeIndices->indices.size() == 0)
        std::cout << "Could not find a plane in the scene." << std::endl;
    else {
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Copy the points of the plane to a new cloud.

        extract.setInputCloud(cloud);
        extract.setIndices(planeIndices);
        extract.setNegative(true);
        extract.filter(*plane_before);

            segmentation.setInputCloud(plane_before);
            segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setAxis(Eigen::Vector3f(1,0,0));
            segmentation.setDistanceThreshold(0.01);
            segmentation.setOptimizeCoefficients(false);
            segmentation.segment(*planeIndices, *coefficients);

            if (planeIndices->indices.size() == 0)
                std::cout << "Could not find a plane in the scene." << std::endl;
            else {
                pcl::ExtractIndices<pcl::PointXYZ> extract2;

                extract2.setInputCloud(plane_before);
                extract2.setIndices(planeIndices);
                extract2.setNegative(true);
                extract2.filter(*plane_before2);

                segmentation.setInputCloud(plane_before2);
                segmentation.setModelType(pcl::SACMODEL_PLANE);
                segmentation.setMethodType(pcl::SAC_RANSAC);
                segmentation.setDistanceThreshold(0.01);
                segmentation.setOptimizeCoefficients(false);
                segmentation.segment(*planeIndices, *coefficients);

                if (planeIndices->indices.size() == 0)
                    std::cout << "Could not find a plane in the scene." << std::endl;
                else {
                    pcl::ExtractIndices<pcl::PointXYZ> extract2;

                    extract2.setInputCloud(plane_before2);
                    extract2.setIndices(planeIndices);
                    extract2.setNegative(true);
                    extract2.filter(*plane);






                    // Creating the KdTree object for the search method of the extraction
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                    tree->setInputCloud (plane);

                    std::vector<pcl::PointIndices> cluster_indices;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                    ec.setClusterTolerance (0.02); // 2cm
                    ec.setMinClusterSize (1000);
                    ec.setMaxClusterSize (25000);
                    ec.setSearchMethod (tree);
                    ec.setInputCloud (plane);
                    ec.extract (cluster_indices);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);

                    int j = 0;
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
                    {
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                            cloud_cluster->points.push_back (plane->points[*pit]); //*
                        cloud_cluster->width = cloud_cluster->points.size ();
                        cloud_cluster->height = 1;
                        cloud_cluster->is_dense = true;

                        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
                        std::stringstream ss;
                        objects = cloud_cluster;

                        j++;
                    }
                    viewerObjects.showCloud(objects, "object_cloud");
                    while (!viewerObjects.wasStopped())
                    {
                        // Do nothing but wait.
                    }

                }

            }
    }

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
		ros::Publisher pub_plane_model = n.advertise<sensor_msgs::PointCloud2>("plane_model",1000);
		sensor_msgs::PointCloud2 plane_out_msg;
		pcl::toROSMsg(*plane_out, plane_out_msg);
		pub_plane_model.publish(plane_out_msg);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
