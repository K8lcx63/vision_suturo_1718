#ifndef VISION_GLOBALS_H
#define VISION_GLOBALS_H

const char* SIM_KINECT_POINTS_FRAME = "/head_mount_kinect/depth_registered/points";
const char* REAL_KINECT_POINTS_FRAME = "/kinect_head/depth_registered/points";



gazebo_msgs::GetModelState getmodelstate;
ros::ServiceClient client;
std::string
        error_message; // Wird durch den Object Position Service mit ausgegeben

pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_global(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr objects_global(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals_global(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointNormal>::Ptr pointnormals_global(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_global(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr objects_rotated_global(new pcl::PointCloud<pcl::PointXYZ>);

geometry_msgs::PointStamped centroid_stamped;

#endif // VISION_GLOBALS_H