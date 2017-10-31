#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, callback);
	ros::spin();

	return 0;
}
