#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <pcl/impl/point_types.hpp>

unsigned int input_noise_threshold = 42;

bool simulation;

void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect);

geometry_msgs::PointStamped findCenterGazebo();

geometry_msgs::PointStamped findCenter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud);

pcl::PointCloud<pcl::Normal>::Ptr estimateSurfaceNormals(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input);

bool objectIsStanding(pcl::PointCloud<pcl::PointXYZ>::Ptr object);

/**
 ** Find the object!
**/
void findCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr kinect) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_y(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_x(
            new pcl::PointCloud<pcl::PointXYZ>);  // Initializes clouds for the
    // PassThroughFilter
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);

    if (kinect->points.size() <
        input_noise_threshold)  // if PR2 is not looking at anything
    {
        ROS_ERROR("INPUT CLOUD EMPTY (PR2: \"OH, MY GOD! I AM BLIND!\"");
        error_message = "Cloud empty. ";
        centroid_stamped = findCenterGazebo();  // Use gazebo data instead
    } else {
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

        if (cloud->points.size() == 0) {
            ROS_ERROR("NO CLOUD AFTER FILTERING");
            error_message = "Cloud was empty after filtering. ";
            centroid_stamped = findCenterGazebo();  // Use gazebo data instead
        } else {
            // ROS_INFO("FINDING PLANE");
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::SACSegmentation<pcl::PointXYZ> segmentation;
            segmentation.setInputCloud(cloud);
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            segmentation.setMethodType(pcl::SAC_RANSAC);
            segmentation.setDistanceThreshold(0.01);  // Distance to model points
            segmentation.setOptimizeCoefficients(true);
            segmentation.segment(*planeIndices, *coefficients);

            if (planeIndices->indices.size() == 0) {
                ROS_ERROR("NO PLANE FOUND");
                error_message = "No plane found. ";
                centroid_stamped = findCenterGazebo();  // Use gazebo data instead
            } else {
                // ROS_INFO("EXTRACT CLUSTER");
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud);
                extract.setIndices(planeIndices);
                extract.setNegative(true);
                extract.filter(*objects);

                if (objects->points.size() == 0) {
                    ROS_ERROR("EXTRACTED CLUSTER IS EMPTY");
                    error_message = "Final extracted cluster was empty. ";
                    centroid_stamped = findCenterGazebo();  // Use gazebo data instead
                }

                centroid_stamped = findCenter(objects);

                // clouds for saving
                kinect_global = kinect;
                objects_global = objects;
            }
        }
    }
}

geometry_msgs::PointStamped
findCenterGazebo()  // Only called if something went wrong in findCluster()
{
    if (simulation)  // Check if this is a simulation. This function is useless if
        // it isn't :(
    {
        ROS_WARN("Something went wrong! Using gazebo data...");
        error_message.append("Gazebo data has been used instead. ");
        client.call(getmodelstate);  // Call client and fill the data
        centroid_stamped.point.x = getmodelstate.response.pose.position.x;
        centroid_stamped.point.y = getmodelstate.response.pose.position.y;
        centroid_stamped.point.z = getmodelstate.response.pose.position.z;
        centroid_stamped.header.frame_id = "world";  // gazebo uses the world frame!
    } else {
        ROS_ERROR(
                "Something went wrong! Gazebo data can't be used: This is not a "
                        "simulation.");
    }

    return centroid_stamped;
}

geometry_msgs::PointStamped findCenter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud) {
    if (object_cloud->points.size() != 0) {
        int cloud_size = object_cloud->points.size();

        float sum_x = 0, sum_y = 0, sum_z = 0;

        for (int i = 0; i < cloud_size; i++) {
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
    } else {
        ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
    }
}

pcl::PointCloud<pcl::Normal>::Ptr estimateSurfaceNormals(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
    ROS_INFO("ESTIMATING SURFACE NORMALS");

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
            new pcl::PointCloud<pcl::Normal>);

    ne.setRadiusSearch(0.03);  // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);
    return cloud_normals;
}

bool objectIsStanding(pcl::PointCloud<pcl::PointXYZ>::Ptr object) {
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

#endif  // VISION_PERCEPTION_H
