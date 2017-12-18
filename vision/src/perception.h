#ifndef VISION_PERCEPTION_H
#define VISION_PERCEPTION_H

#include <Eigen/Geometry>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include "saving.h"

unsigned int input_noise_threshold = 342;
bool simulation;

/** Shorthand **/
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::Normal>::Ptr PointCloudNormalPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointIndices::Ptr PointIndices;


/** Function Headers **/

void findCluster(const PointCloudXYZPtr kinect);

geometry_msgs::PointStamped findCenterGazebo();

geometry_msgs::PointStamped findCenter(const PointCloudXYZPtr object_cloud);

PointCloudNormalPtr estimateSurfaceNormals(PointCloudXYZPtr input);

PointCloudNormalPtr
createPointNormals(PointCloudXYZPtr input, PointCloudNormalPtr normals);

int isStanding();

PointCloudXYZPtr rotatePointCloud(PointCloudXYZPtr cloud);

PointIndices estimatePlaneIndices(PointCloudXYZPtr input);

PointCloudXYZPtr extractCluster(PointCloudXYZPtr input, PointIndices indices, bool negative);

PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane);

void createCovarianceMatrix(PointCloudXYZ input,
                            Eigen::Matrix3f covariance_matrix);

PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z);

PointCloudXYZPtr mlsFilter(PointCloudXYZPtr input);

PointCloudXYZPtr voxelGridFilter(PointCloudXYZPtr input);



/** -------------------------- BEGIN OF IMPLEMENTATION ---------------------- **/

/**
 * Find the object!
 * @param kinect
 */
void findCluster(const PointCloudXYZPtr kinect) {

    // the 'f' in the identifier stands for filtered

    PointCloudXYZPtr cloud_plane(new PointCloudXYZ), cloud_cluster(new PointCloudXYZ), cloud_cluster2(new PointCloudXYZ), cloud_f(
            new PointCloudXYZ), cloud_3df(
            new PointCloudXYZ), cloud_voxelgridf(new PointCloudXYZ), cloud_mlsf(new PointCloudXYZ), cloud_prism(new PointCloudXYZ), result(
            new PointCloudXYZ);

    PointIndices plane_indices(new pcl::PointIndices), plane_indices2(new pcl::PointIndices), prism_indices(new pcl::PointIndices);

    if (kinect->points.size() <
        input_noise_threshold) // if PR2 is not looking at anything
    {
        ROS_ERROR("Input from kinect is empty");
        error_message = "Cloud empty. ";
        centroid_stamped = findCenterGazebo(); // Use gazebo data instead
    } else {
        ROS_INFO("Starting Cluster extraction");

        cloud_3df = apply3DFilter(kinect, 0.4, 0.4, 1.5); // passthrough filter
        cloud_voxelgridf = voxelGridFilter(cloud_3df); // voxel grid filter
        cloud_mlsf = mlsFilter(cloud_voxelgridf); // moving least square filter
        cloud_f = cloud_mlsf; // cloud_f set after last filtering function is applied
        plane_indices = estimatePlaneIndices(cloud_f); // estimate plane indices
        cloud_cluster = extractCluster(cloud_f, plane_indices, true); // extract object
        plane_indices2 = estimatePlaneIndices(cloud_cluster);
        cloud_cluster2 = extractCluster(cloud_cluster, plane_indices2, true); // double extract object
        cloud_plane = extractCluster(cloud_f, plane_indices, false); // extract plane
        prism_indices = prismSegmentation(cloud_cluster, cloud_plane);
        cloud_prism = extractCluster(cloud_cluster, prism_indices, true);

        /** Speichere Zwischenergebenisse **/

        /**
        savePointCloudXYZNamed(cloud_3df, "1_cloud_3d_filtered");
        savePointCloudXYZNamed(cloud_voxelgridf, "2_cloud_voxelgrid_filtered");
        savePointCloudXYZNamed(cloud_mlsf, "3_cloud_mls_filtered");
        savePointCloudXYZNamed(cloud_cluster, "4_cloud_cluster");
        savePointCloudXYZNamed(cloud_plane, "5_cloud_plane");
        savePointCloudXYZNamed(cloud_prism, "6_cloud_prism");
        savePointCloudXYZNamed(cloud_cluster2, "7_cluster_2");
        **/



        ROS_INFO("%sExtraction OK", GREEN_MSG_COL);

        if (cloud_cluster->points.size() == 0) {
            ROS_ERROR("Extracted Cluster is empty");
            error_message = "Final extracted cluster was empty. ";
            centroid_stamped = findCenterGazebo(); // Use gazebo data instead
        }

        error_message = "";
        centroid_stamped = findCenter(cloud_cluster);

        // clouds for saving
        kinect_global = kinect;
        objects_global = cloud_cluster;

    }
}


/**
 * Returns a fake point (not estimated) with its origin in the model in simulation.
 * It is only called, when findCluster() cannot find a point in simulation
 * @return
 */
geometry_msgs::PointStamped
findCenterGazebo() {
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

/**
 * finding the geometrical center of a given pointcloud
 * @param object_cloud
 * @return
 */
geometry_msgs::PointStamped findCenter(const PointCloudXYZPtr object_cloud) {
    if (object_cloud->points.size() != 0) {
        int cloud_size = object_cloud->points.size();

        Eigen::Vector4f centroid;

        pcl::compute3DCentroid(*object_cloud, centroid);

        centroid_stamped.point.x = centroid.x();
        centroid_stamped.point.y = centroid.y();
        centroid_stamped.point.z = centroid.z();

        ROS_INFO("%sCURRENT CLUSTER CENTER\n", "\x1B[32m");
        ROS_INFO("\x1B[32mX: %f\n", centroid_stamped.point.x);
        ROS_INFO("\x1B[32mY: %f\n", centroid_stamped.point.y);
        ROS_INFO("\x1B[32mZ: %f\n", centroid_stamped.point.z);
        centroid_stamped.header.frame_id = "/head_mount_kinect_ir_optical_frame";

        return centroid_stamped;
    } else {
        ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
    }
}

/**
 * estimating surface normals
 * @param input
 * @return
 */
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

/**
 * compute if a certain object is standing (mostly _global pointclouds are used)
 * @return
 */
int isStanding() {

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tammo/catkin_ws/src/vision_suturo_1718/vision/data/eistee_mesh.pcd",
                                            *mesh_global) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read eistee_mesh.pcd \n");
    }

    if (true) { // TODO: Implement pose estimation
        return 1;
    }

    return 0;
}

/**
 * Rotates a pointcloud. parameters have to be manually set
 * @param cloud
 * @return
 */
PointCloudXYZPtr rotatePointCloud(PointCloudXYZPtr cloud) {

    float theta = M_PI; // the angle of rotation in radians

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    transform.translation() << -2.0, 0.0, 0.0;

    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    transform.rotate(Eigen::AngleAxisf(-(theta / 2), Eigen::Vector3f::UnitY()));

    PointCloudXYZPtr transformed_cloud(new PointCloudXYZ());

    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    savePointCloudXYZ(transformed_cloud);
    return transformed_cloud;
}

/**
 * apply a passthrough Filter to all dimensions, reducing points and
 * narrowing Field of Vision
 * @param input
 * @param x
 * @param y
 * @param z
 * @return
 */
PointCloudXYZPtr apply3DFilter(PointCloudXYZPtr input, float x, float y,
                               float z) {

    //TODO test filtering here
    ROS_INFO("Starting passthrough filter");
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

    if (input_after_xyz->points.size() == 0) {
        ROS_ERROR("Cloud empty after passthrough filtering");
        error_message = "Cloud was empty after filtering. ";
        centroid_stamped = findCenterGazebo(); // Use gazebo data instead
    }

    return input_after_xyz;
}

/**
 * estimating plane indices
 * @param input
 * @return
 */
PointIndices estimatePlaneIndices(PointCloudXYZPtr input) {
    ROS_INFO("Starting plane indices estimation");
    PointIndices planeIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setInputCloud(input);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);


    if (planeIndices->indices.size() == 0) {
        ROS_ERROR("No plane (indices) found");
        error_message = "No plane found. ";
        centroid_stamped = findCenterGazebo(); // Use gazebo data instead
    }

    return planeIndices;
}

/**
 * prism segmentation
 * @param input_cloud
 * @param plane
 * @return
 */
PointIndices prismSegmentation(PointCloudXYZPtr input_cloud, PointCloudXYZPtr plane) {
    PointCloudXYZPtr plane_hull = plane;
    ROS_INFO("Starting prism segmentation...");
    pcl::ConvexHull<pcl::PointXYZ> hull;
    PointIndices prism_indices(new pcl::PointIndices);
    hull.setInputCloud(input_cloud);
    hull.reconstruct(*plane_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(input_cloud);
    prism.setInputPlanarHull(plane);
    prism.setHeightLimits(0, 2); // Get everything up to 2 meters above the plane
    prism.segment(*prism_indices);

    return prism_indices;
}

/**
 * extract a pointcloud by indices from an input pointcloud
 * @param input
 * @param indices
 * @param negative
 * @return
 */
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

/**
 * Filtering the input cloud with a moving least squares algorithm
 * @param input
 * @return
 */
PointCloudXYZPtr mlsFilter(PointCloudXYZPtr input) {
    PointCloudXYZPtr result(new PointCloudXYZ);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(input);
    mls.setPolynomialOrder(1); // TODO lookup this function for correct input
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(mls_points);

    for (int i = 0; i < mls_points.size(); i++) {
        pcl::PointXYZ point;

        point.x = mls_points.at(i).x;
        point.y = mls_points.at(i).y;
        point.z = mls_points.at(i).z;
        result->push_back(point);
    }
    return result;
}


/**
 * Filtering the input cloud with a voxel grid
 * @param input
 * @return
 */
PointCloudXYZPtr voxelGridFilter(PointCloudXYZPtr input) {
    PointCloudXYZPtr result(new PointCloudXYZ);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*result);
    return result;
}


#endif // VISION_PERCEPTION_H