#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Point.h"
#include "actionlib/server/simple_action_server.h"

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

	// ServiceClient für das Abrufen der Eistee-Position aus Gazebo.
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); 
	gazebo_msgs::GetModelState getmodelstate;

	getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

	if (client.call(getmodelstate))
	{
		ROS_INFO("Der Service wurde abgerufen.");
		ROS_INFO("X= %lf ;", getmodelstate.response.pose.position.x); // Testweise X ausgeben.
		geometry_msgs::Point point; // Der Punkt, der später übergeben wird.
		point.x = getmodelstate.response.pose.position.x;
		point.y = getmodelstate.response.pose.position.y;
		point.z = getmodelstate.response.pose.position.z;
	}
	else
	{
		ROS_ERROR("Der Service konnte nicht abgerufen werden.");
	}

	// SimpleActionServer für das Übergeben der Eistee-Position aus Gazebo.
	// actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;

	ros::spin();
	return 0;
}
