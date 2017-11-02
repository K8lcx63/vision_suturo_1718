#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <gazebo_msgs/GetModelState.h>
#include "gazebo_msgs/ModelState.h"

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]"); // Gib irgendwas aus, funktioniert nicht so richtig
}

int main(int argc, char **argv)
{
	// Subscriber für das points-Topic des Kinect-Sensors.
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/head_mount_kinect/depth_registered/points", 1000, callback);

	// ServiceClient für den get_model_state-Service von gazebo.
	// ros::ServiceClient client = n.serviceClient<sensor_msgs::PointCloud2>("gazebo/get_model_state");
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
	gazebo_msgs::GetModelState getmodelstate;
	//gazebo_msgs::ModelState modelstate;
	//modelstate.model_name = "eistee";
	//getmodelstate.request.model_state = modelstate;

	if (client.call(getmodelstate))
	{
		ROS_INFO("Der Service wurde abgerufen.");
	}
	else
	{
		ROS_ERROR("Der Service konnte nicht abgerufen werden.");
		return 1;
	}

	ros::spin();
	return 0;
}
