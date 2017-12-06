#ifndef VISION_GLOBALS_H
#define VISION_GLOBALS_H

gazebo_msgs::GetModelState getmodelstate;
ros::ServiceClient client;
std::string
        error_message;  // Wird durch den Object Position Service mit ausgegeben

pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_global;
pcl::PointCloud<pcl::PointXYZ>::Ptr objects_global;
pcl::PointCloud<pcl::Normal>::Ptr normals_global;
pcl::PointCloud<pcl::Normal>::Ptr normals_estimation_1;
pcl::PointCloud<pcl::Normal>::Ptr normals_estimation_2;
pcl::PointCloud<pcl::Normal>::Ptr normals_pose;

geometry_msgs::PointStamped centroid_stamped;

#endif  // VISION_GLOBALS_H
