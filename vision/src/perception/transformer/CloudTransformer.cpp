//
// Created by tammo on 21.01.18.
//



#include "CloudTransformer.h"
#include "../../saving/saving.h"


CloudTransformer::CloudTransformer(ros::NodeHandle nh) : nh_(nh) {

        buffer_.reset(new PointCloudRGB); // (new sensor_msgs::PointCloud2)
        buffer_->header.frame_id = "odom_combined";



}

PointCloudRGBPtr CloudTransformer::removeBelowPlane(PointCloudRGBPtr input) {
    PointCloudRGBPtr cloud_odom_combined(new PointCloudRGB);

    cloud_odom_combined = CloudTransformer::transform(input, "odom_combined", "head_mount_kinect_ir_optical_frame");
    ROS_INFO("TRANSFORMED!");

    // Find the bottom plane
    PointIndices planeIndices(new pcl::PointIndices);
    ROS_INFO("FINDING PLANE");
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_odom_combined);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(500); // Default is 50 and could be problematic
    segmentation.setAxis(Eigen::Vector3f(0, 0, 1));
    segmentation.setEpsAngle(5.0f * (M_PI / 180.0f)); // plane can be within 30 degrees of X-Z plane
    segmentation.setDistanceThreshold(0.02);  // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);

    PointCloudRGBPtr plane(new PointCloudRGB);
    plane = extractCluster(cloud_odom_combined, planeIndices, false); // extract the plane

    // savePointCloudXYZNamed(plane, "ground_plane");

    float min_height = plane->points[0].z;
    for (int i = 1; i < plane->points.size(); i++) { // Search for the lowest point on the plane
        if (plane->points[i].z < min_height) {
            min_height = plane->points[i].z;
        }
    }

    PointCloudRGBPtr result(new PointCloudRGB);
    ROS_INFO("Plane height: %f", min_height);
    ROS_INFO("STARTING PASSTHROUGH FILTER");
    pcl::PassThrough<pcl::PointXYZRGB> pass_above; // Filter out all points below the min_height
    pass_above.setInputCloud(cloud_odom_combined);
    pass_above.setFilterFieldName("z");
    pass_above.setFilterLimits(min_height, 5.00);
    pass_above.setKeepOrganized(false);
    pass_above.filter(*result);

    //savePointCloudXYZNamed(result, "aaaaa");

    return result; // THIS POINTCLOUD IS STILL IN ODOM_COMBINED!


};

PointCloudRGBPtr CloudTransformer::transform(const PointCloudRGBPtr cloud, std::string target_frame,
                           std::string source_frame) // sensor_msgs::PointCloud2ConstPtr&
{
    ROS_INFO("TRYING TO TRANSFORM...");
    try {
        // Usually: target_frame = "odom_combined", source_frame = "head_mount_kinect_ir_optical_frame"
        listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform(target_frame, source_frame, ros::Time(0), stamped_transform_);
        tf::transformTFToEigen(stamped_transform_, transform_eigen_);
        pcl::transformPointCloud(*cloud, *buffer_, transform_eigen_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    //savePointCloudXYZNamed(cloud, "before_transforming");
    //savePointCloudXYZNamed(buffer_, "transformed");
    ROS_INFO("TRANSFORMED!");
    return buffer_;
};
