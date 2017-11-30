#include "actionlib/server/simple_action_server.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "sensor_msgs/PointCloud2.h"
#include "object_detection/ObjectDetection.h"
#include "object_detection/VisObjectInfo.h"
#include "vision_msgs/ObjectInformation.h"
#include "vision_msgs/GetObjectInfo.h"

// includes for pcl

#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>


/** global variables **/
gazebo_msgs::GetModelState getmodelstate;
geometry_msgs::PointStamped centroid_stamped;

pcl::PointCloud<pcl::PointXYZ>::Ptr kinect_global;
pcl::PointCloud<pcl::PointXYZ>::Ptr objects_global;
pcl::PointCloud<pcl::Normal>::Ptr normals_global;

ros::ServiceClient client;

std::string error_message; // Wird durch den Object Position Service mit ausgegeben

bool simulation;

unsigned int filenr;
unsigned int input_noise_threshold = 42;
int one_loop = 0;



/** function heads **/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

geometry_msgs::PointStamped
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

geometry_msgs::PointStamped findCenterGazebo();

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

bool getObjectPosition(object_detection::VisObjectInfo::Request &req,
                       object_detection::VisObjectInfo::Response &res);

bool getObjectPose(vision_msgs::GetObjectInfo::Request &req,
                   vision_msgs::GetObjectInfo::Response &res);

bool objectIsStanding(pcl::PointCloud<pcl::PointXYZ>::Ptr object);

pcl::PointCloud<pcl::Normal>::Ptr estimateSurfaceNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr input);


/** main function **/
int main(int argc, char **argv)
{
    // Subscriber für das points-Topic des Kinect-Sensors.
    ros::init(argc, argv, "vision_main");

    /** nodehandle, subscribers and publishers**/
    ros::NodeHandle n;

    // Subscriber for the kinect points. Also calls findCluster.
    ros::Subscriber sub_kinect = n.subscribe(
            "/head_mount_kinect/depth_registered/points", 100, &findCluster);

    /** services and clients **/
    // ServiceClient for calling the object position through gazebo
    ros::ServiceClient client =  n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    getmodelstate.request.model_name = "eistee"; // Name des Objekts in Gazebo.

    // Service for returning the object centroid
    ros::ServiceServer point_service =
            n.advertiseService("vision_main/visObjectInfo", getObjectPosition);
    ROS_INFO("%sPOINT SERVICE READY\n", "\x1B[32m");

    // Service for returning if the object has fallen over already
    ros::ServiceServer pose_service =
            n.advertiseService("vision_main/objectPose", getObjectPose);
    ROS_INFO("%sPOSE SERVICE READY\n", "\x1B[32m");


    filenr = 0; // apply numbers for saving pcd files
    centroid_stamped.point.x = 0, centroid_stamped.point.y = 0, centroid_stamped.point.z = 0; // dummy point
    one_loop = 0; // for one loop normal estimation
    ros::Rate r(2.0);



    // Parameter testing stuff. Private node handle.
    //bool simulation;
    //ros::NodeHandle n_private_("~");
    //n_private_.param("simulation", simulation, true);
    //n.param("/vision_base/simulation", simulation, false);

    while (n.ok())
    {
        //ros::Publisher pub_objects = n.advertise<sensor_msgs::PointCloud2>("/vision_main/objects",1000);
        //pub_objects.publish(objects_global);
        //client.call(getmodelstate); // continously call model state

        /** simulation parameter start **/
        /*
        // DOESNT WORK :(
        n.param<bool>("simulation", simulation, "true");
        ROS_INFO("%s", simulation.c_str());

        if (n_private_.getParam("simulation", simulation))
        {
            ROS_INFO("Parameter simulation exists!");
            if(simulation)
            {
                ROS_INFO("Parameter simulation is true!");
            }
            else
            {
                ROS_INFO("Parameter simulation is false!");
            }
        }
        else
        {
            ROS_WARN("Parameter simulation does not exist!");
        }
        */
        /** simulation parameter stop **/

        bool simulation = client.exists(); // Periodically check if this is a simulation
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

/**
 ** Find the object!
**/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_y(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_x(new pcl::PointCloud<pcl::PointXYZ>); // Initializes clouds for the PassThroughFilter
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);


    if (kinect->points.size() < input_noise_threshold) // if PR2 is not looking at anything
    {
        ROS_ERROR("INPUT CLOUD EMPTY (PR2: \"OH, MY GOD! I AM BLIND!\"");
        error_message = "Cloud empty. ";
        centroid_stamped = findCenterGazebo(); // Use gazebo data instead
    }
    else
    {
        ROS_INFO("CLUSTER EXTRACTION STARTED");
        // Objects for storing the point clouds.



        /** Create the filtering object **/
        // Create the filtering object (x-axis)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(kinect);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.5, 0.5);
        pass.setKeepOrganized(false);
        pass.filter(*cloud_x);

        // Create the filtering object (y-axis)
        pass.setInputCloud(cloud_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-1.0, 1.0);
        pass.setKeepOrganized(false);
        pass.filter(*cloud_y);

        // Create the filtering object (z-axis)
        pass.setInputCloud(cloud_y);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.8);
        pass.setKeepOrganized(false);
        pass.filter(*cloud);

        if (cloud->points.size() == 0)
        {
            ROS_ERROR("NO CLOUD AFTER FILTERING");
            error_message = "Cloud was empty after filtering. ";
            centroid_stamped = findCenterGazebo(); // Use gazebo data instead
        }
        else
        {
            // ROS_INFO("FINDING PLANE");
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZ> segmentation;
            segmentation.setInputCloud(cloud);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(0.01); // Distance to model points
            segmentation.setOptimizeCoefficients(true);
            segmentation.segment(*planeIndices, *coefficients);


            if (planeIndices->indices.size() == 0)
            {
                ROS_ERROR("NO PLANE FOUND");
                error_message = "No plane found. ";
                centroid_stamped = findCenterGazebo(); // Use gazebo data instead
            }
            else
            {

                // ROS_INFO("EXTRACT CLUSTER");
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud);
                extract.setIndices(planeIndices);
                extract.setNegative(true);
                extract.filter(*objects);

                if (objects->points.size() == 0)
                {
                    ROS_ERROR("EXTRACTED CLUSTER IS EMPTY");
                    error_message = "Final extracted cluster was empty. ";
                    centroid_stamped = findCenterGazebo(); // Use gazebo data instead
                }

                    centroid_stamped = findCenter(objects);

                    // clouds for saving
                    kinect_global = kinect;
                    objects_global = objects;

                    /** estimate surface normals one time only **/

                    if(one_loop == 0) {
                        normals_global = estimateSurfaceNormals(objects);
                        one_loop++;
                    }
                }

            }
        }
    }








geometry_msgs::PointStamped
findCenter(const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)
{

    if (object_cloud->points.size() != 0)
    {
        int cloud_size = object_cloud->points.size();

        float sum_x = 0, sum_y = 0, sum_z = 0;

        for (int i = 0; i < cloud_size; i++)
        {
            pcl::PointXYZ point = object_cloud->points[i];

            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }

        geometry_msgs::Point centroid;
        centroid_stamped.point.x = sum_x / cloud_size;
        centroid_stamped.point.y = sum_y / cloud_size;
        centroid_stamped.point.z = sum_z / cloud_size;

        ROS_INFO("%sCURRENT CLUSTER CENTER\n", "\x1B[32m");
        ROS_INFO("\x1B[32mX: %f\n", centroid_stamped.point.y);
        ROS_INFO("\x1B[32mY: %f\n", centroid_stamped.point.x);
        ROS_INFO("\x1B[32mZ: %f\n", centroid_stamped.point.z);

        centroid_stamped.header.frame_id = "/head_mount_kinect_ir_optical_frame";
        return centroid_stamped;
    }
    else
    {
        ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
    }

}

geometry_msgs::PointStamped findCenterGazebo() // Only called if something went wrong in findCluster()
{
    if(simulation) // Check if this is a simulation. This function is useless if it isn't :(
    {
        ROS_WARN("Something went wrong! Using gazebo data...");
        error_message.append("Gazebo data has been used instead. ");
        client.call(getmodelstate); // Call client and fill the data
        centroid_stamped.point.x = getmodelstate.response.pose.position.x;
        centroid_stamped.point.y = getmodelstate.response.pose.position.y;
        centroid_stamped.point.z = getmodelstate.response.pose.position.z;
        centroid_stamped.header.frame_id = "world"; // gazebo uses the world frame!
    }
    else
    {
        ROS_ERROR("Something went wrong! Gazebo data can't be used: This is not a simulation.");
    }

    return centroid_stamped;
}

bool getObjectPosition(object_detection::VisObjectInfo::Request &req,
                       object_detection::VisObjectInfo::Response &res)
{
    ROS_INFO("POINT SERVICE CALLED");
    centroid_stamped.header.stamp = ros::Time::now(); // Timestamp
    res.object.position = centroid_stamped; // Result of findCenter or findCenterGazebo
    res.object.error = error_message; // Let planning know if any problems occured


    // when service is called, input cloud (kinect) and output cloud (extracted
    // objects) from findCluster are saved to ./data

    /**
    Tammo: Hier failed der savePCDFileASCII-Kram, wenn man den Service aufruft.
    **/
    // savePointCloud(objects_global, kinect_global);


    return true;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr objects,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr kinect)
{
        ROS_INFO("SAVING FILES");
        std::stringstream ss;
        std::stringstream ss_input;
        std::stringstream ss_normals;



        // automatc save to $HOME/.ros folder

        ss << "./object_" << filenr << ".pcd";
        ss_input << "./kinect_" << filenr << ".pcd";
        ss_normals << "./kinect_normals_" << filenr << ".pcd";


        pcl::io::savePCDFileASCII(ss.str(), *objects);
        pcl::io::savePCDFileASCII(ss_input.str(), *kinect);
        pcl::io::savePCDFileASCII(ss_normals.str(), *normals_global);


    filenr++;

}

bool getObjectPose(vision_msgs::GetObjectInfo::Request &req,
                   vision_msgs::GetObjectInfo::Response &res)
{
    if (objectIsStanding(objects_global) == true)
    {
        res.info.isStanding = true;
        res.info.information = "Objekt steht";
        return true;
    }
    else
    {
        res.info.isStanding = false;
        res.info.information = "Objekt liegt";
        return false;
    }


}

bool objectIsStanding(pcl::PointCloud<pcl::PointXYZ>::Ptr object){

    if (object->points.size() != 0) {
        int cloud_size = object->points.size();

        float sum_x = 0, sum_y = 0, sum_z = 0;

        for (int i = 0; i < cloud_size; i++) {
            pcl::PointXYZ point = object->points[i];

            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }

        if (sum_y > sum_x) {
            return true;
        } else {
            return false;

        }
    }
}


pcl::PointCloud<pcl::Normal>::Ptr estimateSurfaceNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {

    ROS_INFO("ESTIMATING SURFACE NORMALS (ONE TIME ONLY)");

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);


    ne.setRadiusSearch(0.03); // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);
    return cloud_normals;

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
