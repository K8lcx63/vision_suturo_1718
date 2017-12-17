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
#include <ros/ros.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include "FeatureCloud.h"
#include "TemplateAlignment.h"
#include "saving.h"
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

unsigned int input_noise_threshold = 342;
bool simulation;

/** Shorthand for Correspondence grouping **/
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


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

double executeICPwithIA();

PointCloudXYZPtr mlsFilter(PointCloudXYZPtr input);

PointCloudXYZPtr voxelGridFilter(PointCloudXYZPtr input);

int computeCorrespondece();


/** -------------------------- BEGIN OF IMPLEMENTATION ---------------------- **/

/**
 * Find the object!
 * @param kinect
 */
void findCluster(const PointCloudXYZPtr kinect) {
    findCenterGazebo(); // check if simulation is ongoing

    PointCloudXYZPtr kinect1(new PointCloudXYZ), kinect2(new PointCloudXYZ);
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
        cloud = apply3DFilter(kinect, 0.4, 0.4, 1.5); // input, x, y, z -- 1.0y
        if (cloud->points.size() == 0) {
            ROS_ERROR("NO CLOUD AFTER FILTERING");
            error_message = "Cloud was empty after filtering. ";
            centroid_stamped = findCenterGazebo(); // Use gazebo data instead
        } else {

            // Filtering with voxelgrid and mls
            kinect1 = voxelGridFilter(kinect);
            kinect2 = mlsFilter(kinect1);
            cloud = kinect2;

            planeIndices = estimatePlaneIndices(cloud);

            if (planeIndices->indices.size() == 0) {
                ROS_ERROR("NO PLANE FOUND");
                error_message = "No plane found. ";
                centroid_stamped = findCenterGazebo(); // Use gazebo data instead

            } else {

                objects = extractCluster(cloud, planeIndices, true);

                PointCloudXYZPtr plane_cloud = extractCluster(cloud, planeIndices, false); // Get the segmented plane
                PointIndices prism_indices = prismSegmentation(objects, plane_cloud);
                objects = extractCluster(objects, prism_indices, true);

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

            }
        }
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
 * Creates a Pointnormal cloud from a normal cloud and a regular PointXYZ cloud
 * @param input
 * @param normals
 * @return
 */
pcl::PointCloud<pcl::PointNormal>::Ptr
createPointNormals(PointCloudXYZPtr input, PointCloudNormalPtr normals) {
    ROS_INFO("CREATE SURFACE POINTNORMALS");
    pcl::PointCloud<pcl::PointNormal>::Ptr output(
            new pcl::PointCloud<pcl::PointNormal>);
    output->reserve(input->points.size());
    // TODO: Fix assignment of points
    /*
    for (int i = 0; i < input->points.size(); i++) {

        pcl::PointXYZ point_xyz;
        pcl::Normal normals_point;
        pcl::PointNormal point_normal;

        // output->points[i].normal = normals->points[i].normal;
        normals_point.normal_x = normals_point->points[i].normal_x;
        normals_point.normal_y = normals_point->points[i].normal_y;
        normals_point.normal_z = normals_point->points[i].normal_z;
        // output->points[i].curvature = normals->points[i].curvature;

        point_xyz.x = input->points[i].x;
        point_xyz.y = input->points[i].y;
        point_xyz.z = input->points[i].z;
        output->points.push_back(point_xyz, normals_point);
    }
    */
    return output;
}

/**
 * compute if a certain object is standing (mostly _global pointclouds are used)
 * @return
 */
int objectIsStanding() {

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tammo/catkin_ws/src/vision_suturo_1718/vision/data/eistee_mesh.pcd",
                                            *mesh_global) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }

    if (computeCorrespondece() >= 1) {
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

/**
 * estimating plane indices
 * @param input
 * @return
 */
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
 * compute iterative closest point with initial alignment beforehand
 * @return
 */
double executeICPwithIA() {

    // TODO Fix with multiple input files (pcl documentation tutorial)


    // load FeatureClouds
    FeatureCloud template_cloud;
    template_cloud.loadInputCloud("/home/tammo/catkin_ws/src/vision_suturo_1718/vision/data/eistee_mesh.pcd");

    FeatureCloud target_cloud;
    target_cloud.setInputCloud(kinect_global);

    // set template clouds
    TemplateAlignment template_align;

    template_align.addTemplateCloud(template_cloud);
    template_align.setTargetCloud(target_cloud);

    // Find the best template alignment
    TemplateAlignment::Result best_alignment;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // Save the aligned template for visualization
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointCloud<pcl::PointXYZ> tmpl_for_trans = *kinect_global; //template_cloud.getPointCloud();
    pcl::transformPointCloud(tmpl_for_trans, transformed_cloud, best_alignment.final_transformation);
    pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

    // succeedingly do the icp transformation

    PointCloudXYZPtr transformed_cloud_ptr(new PointCloudXYZ);
    *transformed_cloud_ptr = transformed_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformed_cloud_ptr);
    icp.setInputTarget(mesh_global);
    //icp.setMaximumIterations(10);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return icp.getFitnessScore();
}

/**
 * rotate on point cloud to the orientation of the other
 */
void rotateCloud2Cloud() {
    //TODO inspect where it fails here

    Eigen::Vector4f centroid_objects;
    Eigen::Vector4f centroid_mesh;

    pcl::compute3DCentroid(*objects_global, centroid_objects);
    pcl::compute3DCentroid(*mesh_global, centroid_mesh);

    PointCloudXYZPtr objects_point(new PointCloudXYZ);
    PointCloudXYZPtr mesh_point(new PointCloudXYZ);

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

    pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 rot_mat;

    pcl::registration::TransformationEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr te; // FIX: BOOST_SHARED_PTR ERROR, px!=0 failed

    te->estimateRigidTransformation(*objects_global, *objects_global, rot_mat);
    //pcl::transformPointCloud(*objects_global,*mesh_global,rot_mat);
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
    mls.setPolynomialFit(true);
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

/** correspondence grouping from pcl **/
int computeCorrespondece() {

    bool show_keypoints_(false);
    bool show_correspondences_(false);
    bool use_cloud_resolution_(false);
    bool use_hough_(true);
    float model_ss_(0.01f);
    float scene_ss_(0.03f);
    float rf_rad_(0.015f);
    float descr_rad_(0.02f);
    float cg_size_(0.01f);
    float cg_thresh_(5.0f);

    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

    //
    //  Compute Normals
    //
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch(10);
    norm_est.setInputCloud(mesh_global);
    norm_est.compute(*model_normals);

    std::cout << "Size of mesh normals " << model_normals->size() << std::endl;
    norm_est.setInputCloud(kinect_global);
    norm_est.compute(*scene_normals);


    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(mesh_global);
    uniform_sampling.setRadiusSearch(0.01f);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size()
              << std::endl;

    uniform_sampling.setInputCloud(kinect_global);
    uniform_sampling.setRadiusSearch(scene_ss_);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << kinect_global->size() << "; Selected Keypoints: " << scene_keypoints->size()
              << std::endl;


    //
    //  Compute Descriptor for keypoints
    //
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch(descr_rad_);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(mesh_global);
    descr_est.compute(*model_descriptors);

    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(kinect_global);
    descr_est.compute(*scene_descriptors);

    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size(); ++i) {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!pcl_isfinite (scene_descriptors->at(i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] <
                                 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D

    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_rad_);

    rf_est.setInputCloud(model_keypoints);
    rf_est.setInputNormals(model_normals);
    rf_est.setSearchSurface(mesh_global);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(scene_normals);
    rf_est.setSearchSurface(kinect_global);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(cg_size_);
    clusterer.setHoughThreshold(cg_thresh_);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize(rototranslations, clustered_corrs);

    return rototranslations.size();

}

#endif // VISION_PERCEPTION_H