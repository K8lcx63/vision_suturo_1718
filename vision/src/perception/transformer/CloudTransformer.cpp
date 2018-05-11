//
// Created by tammo on 21.01.18.
//



#include "CloudTransformer.h"
#include "../../saving/saving.h"


CloudTransformer::CloudTransformer(ros::NodeHandle nh) : nh_(nh) {

        buffer_.reset(new PointCloudRGB); // (new sensor_msgs::PointCloud2)
        buffer_->header.frame_id = "odom_combined";

}

/**
 * Finds the main plane (-> table, etc.) and extracts only the points above that plane.
 * @param input PointCloud
 * @return Extracted PointCloud
 */
PointCloudRGBPtr CloudTransformer::extractAbovePlane(PointCloudRGBPtr input) {
    ROS_INFO("Removing points below the ground plane...");
    PointCloudRGBPtr cloud_odom_combined(new PointCloudRGB);

    cloud_odom_combined = CloudTransformer::transform(input, "base_link", "head_mount_kinect_rgb_optical_frame");

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
    segmentation.setEpsAngle(5.0f * (M_PI / 180.0f)); // plane can be within 5 degrees of X-Z plane
    segmentation.setDistanceThreshold(0.02);  // Distance to model points
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);

    PointCloudRGBPtr plane(new PointCloudRGB);
    plane = extractCluster(cloud_odom_combined, planeIndices, false); // extract the plane

    //ROS_INFO("TRYING TO SAVE THE GROUND PLANE");
    //savePointCloudRGBNamed(plane, "ground_plane");

    // Calculate min and max values of the main plane.

    ROS_INFO("BEFORE CALCULATING MIN AND MAX VALUES");
    float min_x = plane->points[0].x;
    float min_y = plane->points[0].y;
    float min_z = plane->points[0].z;

    float max_x = plane->points[0].x;
    float max_y = plane->points[0].y;

    for (int i = 1; i < plane->points.size(); i++) {
        if (plane->points[i].x < min_x) {
            min_x = plane->points[i].x;
        }
        if (plane->points[i].y < min_y) {
            min_y = plane->points[i].y;
        }
        if (plane->points[i].z < min_z) {
            min_z = plane->points[i].z;
        }
        if (plane->points[i].x > max_x) {
            max_x = plane->points[i].x;
        }
        if (plane->points[i].y > max_y) {
            max_y = plane->points[i].y;
        }
    }

    PointCloudRGBPtr cloud_without_plane(new PointCloudRGB);
    PointCloudRGBPtr result(new PointCloudRGB);
    cloud_without_plane = extractCluster(cloud_odom_combined, planeIndices, true);
    // savePointCloudRGBNamed(cloud_without_plane, "without_plane");

    ROS_INFO("Plane height: %f", min_z);
    ROS_INFO("min_x: %f", min_x);
    ROS_INFO("max_x: %f", max_x);
    ROS_INFO("min_y: %f", min_y);
    ROS_INFO("max_y: %f", max_y);

    // Only add points to the result point cloud that fulfill the min and max values
    for(int a = 0; a < cloud_without_plane->points.size(); a++){
        if(     cloud_without_plane->points[a].x >= min_x &&
                cloud_without_plane->points[a].x <= max_x &&
                cloud_without_plane->points[a].y >= min_y &&
                cloud_without_plane->points[a].y <= max_y &&
                cloud_without_plane->points[a].z >= min_z   ){
            result->points.push_back(cloud_without_plane->points[a]);
        }
    }
    result->width = 1;
    result->height = result->points.size();


    PointCloudRGBPtr result_transformed_back (new PointCloudRGB);
    result_transformed_back = CloudTransformer::transform(result, "head_mount_kinect_rgb_optical_frame", "base_link");
    return result_transformed_back;


};

/**
 * Transforms a PointCloud into another frame.
 * @param cloud
 * @param target_frame
 * @param source_frame
 * @return Transformed PointCloud
 */
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
