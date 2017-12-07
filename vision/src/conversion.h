//
// Created by tammo on 06.12.17.
//

#ifndef VISION_CONVERSION_H
#define VISION_CONVERSION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

/**
 * input: PointCloud<PointXYZ>::Ptr
 * output: cv::Mat
 */

cv::Mat PointCloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {

    cv::Mat output;
    for (int i = 0; i < input->points.size(); i++) {
        output.at<float>(0, i) = input->points.at(i).x;
        output.at<float>(1, i) = input->points.at(i).y;
        output.at<float>(2, i) = input->points.at(i).z;
    }

    return output;

}

#endif //VISION_CONVERSION_H
