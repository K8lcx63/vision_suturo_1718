#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H

#include <Eigen/Geometry>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PointStamped.h>

#include "saving.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointIndices::Ptr PointIndices;

unsigned int input_noise_threshold = 342;

bool simulation;

void findCluster(const PointCloudXYZPtr kinect);

geometry_msgs::PointStamped findCenterGazebo();

geometry_msgs::PointStamped findCenter(const PointCloudXYZPtr object_cloud);

PointCloudNormalPtr estimateSurfaceNormals(PointCloudXYZPtr input);

pcl::PointCloud<pcl::PointNormal>::Ptr
createPointNormals(PointCloudXYZPtr input, PointCloudNormalPtr normals);

int objectIsStanding();

PointCloudXYZPtr rotatePointCloud(PointCloudXYZPtr cloud);

PointIndices estimatePlaneIndices(PointCloudXYZPtr input);

PointCloudXYZPtr extractCluster(PointCloudXYZPtr input, PointIndices indices, bool negative);

PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane);

void createCovarianceMatrix(PointCloudXYZ input,
                            Eigen::Matrix3f covariance_matrix);

PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z);
double executeICP(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out);


/**
 ** Find the object!
**/
void findCluster(const PointCloudXYZPtr kinect) {
    PointCloudXYZPtr cloud(new PointCloudXYZ); // for passthroughfilter
    PointCloudXYZPtr objects(new PointCloudXYZ);
    PointCloudXYZPtr result(new PointCloudXYZ);
    PointIndices planeIndices(new pcl::PointIndices);

    if (kinect->points.size() <
        input_noise_threshold) // if PR2 is not looking at anything
    {
        ROS_ERROR("INPUT CLOUD EMPTY (PR2: \"OH, MY GOD! I AM BLIND!\"");
        error_message = "Cloud empty. ";
        centroid_stamped = findCenterGazebo(); // Use gazebo data instead
    } else {
        ROS_INFO("CLUSTER EXTRACTION STARTED");
        // Objects for storing the point clouds.

        // apply passthroughFilter on all Axes (Axis?)
        cloud = apply3DFilter(kinect, 0.5, 1.5, 1.8); // input, x, y, z -- 1.0y
        if (cloud->points.size() == 0) {
            ROS_ERROR("NO CLOUD AFTER FILTERING");
            error_message = "Cloud was empty after filtering. ";
            centroid_stamped = findCenterGazebo(); // Use gazebo data instead
        } else {

            planeIndices = estimatePlaneIndices(cloud);

            if (planeIndices->indices.size() == 0) {
                ROS_ERROR("NO PLANE FOUND");
                error_message = "No plane found. ";
                centroid_stamped = findCenterGazebo(); // Use gazebo data instead

            } else {

                objects = extractCluster(cloud, planeIndices, true);

                //

                PointCloudXYZPtr plane_cloud = extractCluster(cloud, planeIndices, false); // Get the segmented plane
                PointIndices prism_indices = prismSegmentation(objects, plane_cloud);
                objects = extractCluster(objects, prism_indices, true);

                //


                ROS_INFO("EXTRACTION OK");

                if (objects->points.size() == 0) {
                    ROS_ERROR("EXTRACTED CLUSTER IS EMPTY");
                    error_message = "Final extracted cluster was empty. ";
                    centroid_stamped = findCenterGazebo(); // Use gazebo data instead
                }

                error_message = "";
                centroid_stamped = findCenter(objects);

                // clouds for saving
                kinect_global = kinect;
                objects_global = objects;
                objects_rotated_global = rotatePointCloud(objects_global);

            }
        }
    }
}

geometry_msgs::PointStamped
findCenterGazebo() // Only called if something went wrong in findCluster()
{
    if (simulation) // Check if this is a simulation. This function is useless if
        // it isn't :(
    {
        ROS_WARN("Something went wrong! Using gazebo data...");
        error_message.append("Gazebo data has been used instead. ");
        client.call(getmodelstate); // Call client and fill the data
        centroid_stamped.point.x = getmodelstate.response.pose.position.x;
        centroid_stamped.point.y = getmodelstate.response.pose.position.y;
        centroid_stamped.point.z = getmodelstate.response.pose.position.z;
        centroid_stamped.header.frame_id = "world"; // gazebo uses the world frame!
    } else {
        ROS_ERROR("Something went wrong! Gazebo data can't be used: This is not a "
                          "simulation.");
    }

    return centroid_stamped;
}

geometry_msgs::PointStamped findCenter(const PointCloudXYZPtr object_cloud) {
    if (object_cloud->points.size() != 0) {
        int cloud_size = object_cloud->points.size();

        Eigen::Vector4f centroid;

        pcl::compute3DCentroid(*object_cloud, centroid);

        centroid_stamped.point.x = centroid.x();
        centroid_stamped.point.y = centroid.y();
        centroid_stamped.point.z = centroid.z();

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

PointCloudNormalPtr estimateSurfaceNormals(PointCloudXYZPtr input) {
    ROS_INFO("ESTIMATING SURFACE NORMALS");

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    PointCloudNormalPtr cloud_normals(new PointCloudNormal);

    ne.setRadiusSearch(0.03); // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);
    return cloud_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr
createPointNormals(PointCloudXYZPtr input, PointCloudNormalPtr normals) {
    ROS_INFO("CREATE SURFACE POINTNORMALS");
    pcl::PointCloud<pcl::PointNormal>::Ptr output(
            new pcl::PointCloud<pcl::PointNormal>);
    output->reserve(input->points.size());
    // TODO: Fix assignment of points
    for (int i = 0; i < input->points.size(); i++) {

        // output->points[i].normal = normals->points[i].normal;
        output->points[i].normal_x = normals->points[i].normal_x;
        output->points[i].normal_y = normals->points[i].normal_y;
        output->points[i].normal_z = normals->points[i].normal_z;
        // output->points[i].curvature = normals->points[i].curvature;

        output->points[i].x = input->points[i].x;
        output->points[i].y = input->points[i].y;
        output->points[i].z = input->points[i].z;
    }
    return output;
}

int  objectIsStanding() {

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/tammo/.ros/eistee_mesh.pcd", *mesh_global) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    if (executeICP(objects_rotated_global,mesh_global) > 0.0006){
        return 0;
    }

    return 1;
}

PointCloudXYZPtr rotatePointCloud(PointCloudXYZPtr cloud) {

    float theta = M_PI; // the angle of rotation in radians

    /**
  * Method #2: Using Affine3f
  * This method is easier and less error prone
  **/

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();


    transform.translation() << -2.0, 0.0, 0.0;


    // theta radians around z-axis
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    // theta radians around y-axis
    transform.rotate(Eigen::AngleAxisf(-(theta/2), Eigen::Vector3f::UnitY()));



    /** executing transformation **/
    PointCloudXYZPtr transformed_cloud(new PointCloudXYZ());

    // apply transform_1 or transform_2
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    savePointCloudXYZ(transformed_cloud);
    return transformed_cloud;
}

PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z) {
    ROS_INFO("3D FILTER");
    PointCloudXYZPtr input_after_x(new PointCloudXYZ),
            input_after_xy(new PointCloudXYZ), input_after_xyz(new PointCloudXYZ);
    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud(input_after_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.setKeepOrganized(false);
    pass.filter(*input_after_xy);

    // Create the filtering object (z-axis)
    pass.setInputCloud(input_after_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
            0.0, z); // no negative range (the pr2 can't look behind its head)
    pass.setKeepOrganized(false);
    pass.filter(*input_after_xyz);

    return input_after_xyz;
}

PointIndices estimatePlaneIndices(PointCloudXYZPtr input) {
    ROS_INFO("PLANE INDICES");
    PointIndices planeIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setInputCloud(input);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);

    return planeIndices;
}

PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane){
    PointCloudXYZPtr plane_hull = plane;
    ROS_INFO("Starting prism segmentation...");
    pcl::ConvexHull<pcl::PointXYZ> hull;
    PointIndices prism_indices(new pcl::PointIndices);
    hull.setInputCloud (input_cloud);
    hull.reconstruct (*plane_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (input_cloud);
    prism.setInputPlanarHull (plane);
    prism.setHeightLimits (0, 2); // Get everything up to 2 meters above the plane
    prism.segment (*prism_indices);

    return prism_indices;
}

PointCloudXYZPtr extractCluster(PointCloudXYZPtr input, PointIndices indices, bool negative) {
    ROS_INFO("CLUSTER EXTRACTION");
    PointCloudXYZPtr objects(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*objects);
    return objects;
}

double executeICP(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    //icp.setMaximumIterations(10);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation()  << std::endl;
    return icp.getFitnessScore();
}

//TODO inspect where it fails here
void rotateCloud2Cloud()
{
    Eigen::Vector4f centroid_objects;
    Eigen::Vector4f centroid_mesh;

    pcl::compute3DCentroid(*objects_global, centroid_objects);
    pcl::compute3DCentroid(*mesh_global, centroid_mesh);

    PointCloudXYZPtr objects_point (new PointCloudXYZ);
    PointCloudXYZPtr mesh_point (new PointCloudXYZ);

    pcl::PointXYZ point_centroid_obj;
    pcl::PointXYZ point_centroid_mes;



    point_centroid_obj.x = centroid_objects.x();
    point_centroid_obj.x = centroid_objects.y();
    point_centroid_obj.x = centroid_objects.z();

    point_centroid_mes.x = centroid_mesh.x();
    point_centroid_mes.y = centroid_mesh.y();
    point_centroid_mes.z = centroid_mesh.z();

    objects_point->push_back(point_centroid_obj);
    objects_point->push_back(point_centroid_obj);
    objects_point->push_back(point_centroid_obj);

    mesh_point->push_back(point_centroid_mes);
    mesh_point->push_back(point_centroid_mes);
    mesh_point->push_back(point_centroid_mes);

    ROS_INFO("ALL COOL UNTIL Tranformation ESTIMATION");

    pcl::registration::TransformationEstimation< pcl::PointXYZ, pcl::PointXYZ >::Matrix4 rot_mat;

    pcl::registration::TransformationEstimation<pcl::PointXYZ,pcl::PointXYZ>::Ptr te; // FIX: BOOST_SHARED_PTR ERROR, px!=0 failed

    te->estimateRigidTransformation(*objects_global,*objects_global,rot_mat);
    //pcl::transformPointCloud(*objects_global,*mesh_global,rot_mat);
}
#endif // VISION_PERCEPTION_H
