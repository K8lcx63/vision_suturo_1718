//
// Created by tammo on 21.01.18.
//

#ifndef VISION_CLOUDTRANSFORMER_H
#define VISION_CLOUDTRANSFORMER_H

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "../short_types.h"
#include "../perception.h"

#include <string>

class CloudTransformer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    //ros::Publisher pcl_pub_;
    tf::TransformListener listener_;
    tf::StampedTransform stamped_transform_;
    tf::Transform test_transform_;
    Eigen::Affine3d transform_eigen_;
    PointCloudRGBPtr buffer_; // sensor_msgs::PointCloud2::Ptr

public:
    explicit CloudTransformer(ros::NodeHandle nh);
    PointCloudRGBPtr transform(const PointCloudRGBPtr cloud, std::string target_frame,
                               std::string source_frame) ;
    PointCloudRGBPtr extractAbovePlane(PointCloudRGBPtr input) ;

};


#endif //VISION_CLOUDTRANSFORMER_H
