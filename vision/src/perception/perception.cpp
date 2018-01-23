/**
 * old:
 * #include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/registration/transformation_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


#include "short_types.h"
#include "transformer/CloudTransformer.h"
#include "../saving/saving.h"

 */





#include "perception.h"

geometry_msgs::PointStamped centroid_stamped_perc;

std::string error_message_perc;

/**
 * Find the object!
 * @param kinect
 */
std::vector<sensor_msgs::PointCloud2> findCluster(PointCloudRGBPtr kinect) {

    ros::NodeHandle n;
    std::vector<sensor_msgs::PointCloud2> result;

    CloudTransformer transform_cloud(n);

    // the 'f' in the identifier stands for filtered

    PointCloudRGBPtr cloud_plane(new PointCloudRGB), cloud_cluster(new PointCloudRGB), cloud_cluster2(
            new PointCloudRGB), cloud_f(
            new PointCloudRGB), cloud_3df(
            new PointCloudRGB), cloud_voxelgridf(new PointCloudRGB), cloud_mlsf(new PointCloudRGB), cloud_prism(
            new PointCloudRGB), cloud_final(new PointCloudRGB);

    PointIndices plane_indices(new pcl::PointIndices), plane_indices2(new pcl::PointIndices), prism_indices(
            new pcl::PointIndices);


    if (kinect->points.size() <
        500)                              // if PR2 is not looking at anything
    {
        ROS_ERROR("Input from kinect is empty");
        error_message_perc = "Cloud empty. ";
        centroid_stamped_perc = findCenterGazebo();              // Use gazebo data instead
    } else {
        ROS_INFO("Starting Cluster extraction");

        cloud_3df = apply3DFilter(kinect, 0.4, 0.4, 1.5);   // passthrough filter
        cloud_voxelgridf = voxelGridFilter(cloud_3df);      // voxel grid filter
        cloud_mlsf = mlsFilter(cloud_voxelgridf);           // moving least square filter
        cloud_cluster = cloud_mlsf;                         // cloud_f set after last filtering function is applied

        transform_cloud.removeBelowPlane(cloud_cluster);

        // While a segmented plane would be larger than 500 points, segment it.
        bool loop_plane_segmentations = true;
        int amount_plane_segmentations = 0;
        while (loop_plane_segmentations) {
            plane_indices = estimatePlaneIndices(cloud_cluster);
            if (plane_indices->indices.size() > 500)         // is the extracted plane big enough?
            {
                ROS_INFO("plane_indices: %lu", plane_indices->indices.size());
                ROS_INFO("cloud_cluster: %lu", cloud_cluster->points.size());
                cloud_cluster = extractCluster(cloud_cluster, plane_indices, true); // actually extract the object
                amount_plane_segmentations++;
            } else loop_plane_segmentations = false;          // if not big enough, stop looping.
        }
        ROS_INFO("Extracted %d planes!", amount_plane_segmentations);

        cloud_final = outlierRemoval(cloud_cluster);

        /** Speichere Zwischenergebenisse **/

        /**
        savePointCloudRGBNamed(cloud_3df, "1_cloud_3d_filtered");
        savePointCloudRGBNamed(cloud_voxelgridf, "2_cloud_voxelgrid_filtered");
        savePointCloudRGBNamed(cloud_mlsf, "3_cloud_mls_filtered");
        savePointCloudRGBNamed(cloud_cluster, "4_cloud_cluster");
        savePointCloudRGBNamed(cloud_prism, "6_cloud_prism");
        savePointCloudRGBNamed(cloud_cluster2, "7_cluster_2");
        savePointCloudRGBNamed(cloud_final, "result");
        **/


        ROS_INFO("%sExtraction OK", "\x1B[32m");

        if (cloud_cluster->points.size() == 0) {
            ROS_ERROR("Extracted Cluster is empty");
            error_message_perc = "Final extracted cluster was empty. ";
            centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
        }

        error_message_perc = "";


        // convert clustered objects
        sensor_msgs::PointCloud2 pcloud2_msg;
        pcl::toROSMsg(*cloud_final, pcloud2_msg);

        result[0] = pcloud2_msg;                            // add clustered objects to result

        return result;

    }
}

/**
 * Returns a fake point (not estimated) with its origin in the model in simulation.
 * It is only called, when findCluster() cannot find a point in simulation
 * @return
 */
geometry_msgs::PointStamped
findCenterGazebo() {

    return centroid_stamped_perc;
}

/**
 * finding the geometrical center of a given pointcloud
 * @param object_cloud
 * @return
 */
std::vector<geometry_msgs::PointStamped> findCenter(const std::vector<sensor_msgs::PointCloud2> object_clouds_in) {
    std::vector<geometry_msgs::PointStamped> result;

    // convert pointclouds
    std::vector<PointCloudRGBPtr> object_clouds;
    PointCloudRGBPtr temp(new PointCloudRGB);
    for (int i = 0; i < object_clouds_in.size(); i++) {
        pcl::fromROSMsg(object_clouds_in[i], *temp);
        object_clouds[i] = temp;
    }

    // calculate centroids

    PointCloudRGBPtr object_cloud = object_clouds[0];
        if (object_cloud->points.size() != 0) {
            int cloud_size = object_cloud->points.size();

            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(*object_cloud, centroid);

            centroid_stamped_perc.point.x = centroid.x();
            centroid_stamped_perc.point.y = centroid.y();
            centroid_stamped_perc.point.z = centroid.z();

            ROS_INFO("%sCURRENT CLUSTER CENTER\n", "\x1B[32m");
            ROS_INFO("\x1B[32mX: %f\n", centroid_stamped_perc.point.x);
            ROS_INFO("\x1B[32mY: %f\n", centroid_stamped_perc.point.y);
            ROS_INFO("\x1B[32mZ: %f\n", centroid_stamped_perc.point.z);
            centroid_stamped_perc.header.frame_id = "/head_mount_kinect_ir_optical_frame";

            result.push_back(centroid_stamped_perc);
        } else {
            ROS_ERROR("CLOUD EMPTY. NO POINT EXTRACTED");
        }

}

/**
 * estimating surface normals
 * @param input
 * @return
 */
PointCloudNormalPtr estimateSurfaceNormals(PointCloudRGBPtr input) {
    ROS_INFO("ESTIMATING SURFACE NORMALS");

    // PointXYZRGB to PointXYZ

    PointCloudXYZPtr input_xyz (new PointCloudXYZ);

    for (size_t i = 0; i < input->size(); i++){
        input_xyz->points[i].x = input->points[i].x;
        input_xyz->points[i].y = input->points[i].y;
        input_xyz->points[i].z = input->points[i].z;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_xyz);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    PointCloudNormalPtr cloud_normals(new PointCloudNormal);

    ne.setRadiusSearch(0.03); // Use all neighbors in a sphere of radius 3cm

    ne.compute(*cloud_normals);
    return cloud_normals;
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
PointCloudRGBPtr apply3DFilter(PointCloudRGBPtr input, float x, float y,
                               float z) {

    PointCloudRGBPtr result (new PointCloudRGB);


    ROS_INFO("Starting passthrough filter");
    PointCloudRGBPtr input_after_x(new PointCloudRGB),
            input_after_xy(new PointCloudRGB), input_after_xyz(new PointCloudRGB);
    /** Create the filtering object **/
    // Create the filtering object (x-axis)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);

    pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(true);
    pass.filter(*input_after_x);

    // Create the filtering object (y-axis)
    pass.setInputCloud(input_after_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);

    pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(true);
    pass.filter(*input_after_xy);

    // Create the filtering object (z-axis)
    pass.setInputCloud(input_after_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
            0.0, z); // no negative range (the pr2 can't look behind its head)

    pass.setUserFilterValue(0.0f);
    pass.setKeepOrganized(true);
    pass.filter(*input_after_xyz);

    if (input_after_xyz->points.size() == 0) {
        ROS_ERROR("Cloud empty after passthrough filtering");
        error_message_perc = "Cloud was empty after filtering. ";
        centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
    }



    return result;
}

/**
 * estimating plane indices
 * @param input
 * @return
 */
PointIndices estimatePlaneIndices(PointCloudRGBPtr input) {



    ROS_INFO("Starting plane indices estimation");
    PointIndices planeIndices(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;

    segmentation.setInputCloud(input);
    //segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // PERPENDICULAR
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    //segmentation.setMaxIterations(500); // Default is 50 and could be problematic
    //segmentation.setAxis(Eigen::Vector3f(0,1,0));
    //segmentation.setEpsAngle(30.0); // plane can be within 30 degrees of X-Z plane - 30.0f * (M_PI/180.0f)
    segmentation.setDistanceThreshold(0.01); // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);


    if (planeIndices->indices.size() == 0) {
        ROS_ERROR("No plane (indices) found");
        error_message_perc = "No plane found. ";
        centroid_stamped_perc = findCenterGazebo(); // Use gazebo data instead
    }

    return planeIndices;
}

/**
 * prism segmentation
 * @param input_cloud
 * @param plane
 * @return
 */
PointIndices prismSegmentation(PointCloudRGBPtr input_cloud, PointCloudRGBPtr plane) {

    // PointXYZRGB to PointXYZ

    PointCloudXYZPtr input_xyz (new PointCloudXYZ);

    for (size_t i = 0; i < input_cloud->size(); i++){
        input_xyz->points[i].x = input_cloud->points[i].x;
        input_xyz->points[i].y = input_cloud->points[i].y;
        input_xyz->points[i].z = input_cloud->points[i].z;
    }

    PointCloudXYZPtr plane_xyz (new PointCloudXYZ);

    for (size_t i = 0; i < plane->size(); i++){
        plane_xyz->points[i].x = plane->points[i].x;
        plane_xyz->points[i].y = plane->points[i].y;
        plane_xyz->points[i].z = plane->points[i].z;
    }

    PointCloudXYZPtr plane_hull = plane_xyz;
    ROS_INFO("Starting prism segmentation...");
    pcl::ConvexHull<pcl::PointXYZ> hull;
    PointIndices prism_indices(new pcl::PointIndices);
    hull.setInputCloud(input_xyz);
    hull.reconstruct(*plane_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(input_xyz);
    prism.setInputPlanarHull(plane_xyz);
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
PointCloudRGBPtr extractCluster(PointCloudRGBPtr input, PointIndices indices, bool negative) {
    ROS_INFO("CLUSTER EXTRACTION");
    PointCloudRGBPtr result (new PointCloudRGB);

    // PointXYZRGB to PointXYZ

    PointCloudXYZPtr input_xyz (new PointCloudXYZ);

    for (size_t i = 0; i < input->size(); i++){
        input_xyz->points[i].x = input->points[i].x;
        input_xyz->points[i].y = input->points[i].y;
        input_xyz->points[i].z = input->points[i].z;
    }
    PointCloudXYZPtr objects(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_xyz);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.setUserFilterValue(0.0f);
    extract.setKeepOrganized(true);
    extract.filter(*objects);

    for (size_t i = 0; i < objects->size(); i++){
        if (objects->points[i].x != 0.0f &&
            objects->points[i].y != 0.0f &&
            objects->points[i].z != 0.0f){

            result->points[i].x = objects->points[i].x;
            result->points[i].y = objects->points[i].y;
            result->points[i].z = objects->points[i].z;
            result->points[i].r = input->points[i].r;
            result->points[i].g = input->points[i].g;
            result->points[i].b = input->points[i].b;

            // not sure if needed
            result->points[i].rgb = input->points[i].rgb;
            result->points[i].rgba = input->points[i].rgba;

        }
    }


    return result;
}

/**
 * Filtering the input cloud with a moving least squares algorithm
 * @param input
 * @return
 */
PointCloudRGBPtr mlsFilter(PointCloudRGBPtr input) {
    PointCloudRGBPtr result (new PointCloudRGB);

    // PointXYZRGB to PointXYZ

    PointCloudXYZPtr input_xyz (new PointCloudXYZ);

    for (size_t i = 0; i < input->size(); i++){
        input_xyz->points[i].x = input->points[i].x;
        input_xyz->points[i].y = input->points[i].y;
        input_xyz->points[i].z = input->points[i].z;
    }
    int poly_ord = 1;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(input);
    mls.setPolynomialOrder(poly_ord); // the lower the smoother, the higher the more exact
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    mls.process(mls_points);

    for (int i = 0; i < mls_points.size(); i++) {
        pcl::PointXYZRGB point;

        point.x = mls_points.points[i].x;
        point.y = mls_points.points[i].y;
        point.z = mls_points.points[i].z;
        point.r = input->points[i].r;
        point.g = input->points[i].g;
        point.b = input->points[i].b;
        point.rgb = input->points[i].rgb;
        point.rgba = input->points[i].rgba;
        result->push_back(point);
    }

    return result;
}


/**
 * Filtering the input cloud with a voxel grid
 * @param input
 * @return
 */
PointCloudRGBPtr voxelGridFilter(PointCloudRGBPtr input) {
    PointCloudRGBPtr result(new PointCloudRGB);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*result);
    return result;
}

PointCloudRGBPtr outlierRemoval(PointCloudRGBPtr input) {
    PointCloudRGBPtr cloud_filtered(new PointCloudRGB);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr cvfhRecognition(PointCloudRGBPtr input) {
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the CVFH descriptors.
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);

    // Estimate the normals of the object.
    normals = estimateSurfaceNormals(input);

    // New KdTree to search with.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    // CVFH estimation object.
    pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(input);
    cvfh.setInputNormals(normals);
    cvfh.setSearchMethod(kdtree);
    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);
    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(false);

    cvfh.compute(*descriptors);
}