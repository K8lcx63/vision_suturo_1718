//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

//ROS_INFO("Initializing classifier!");
CvNormalBayesClassifier *bayes = new CvNormalBayesClassifier;

/**
 *
 * @param cloud: A partial view PointCloud of an object
 * @param label_index: The label of this object as an index from mesh_enum in perception.cpp
 * @param update: True keeps previous training data
 * @return Whether the training was successful
 */

bool train(PointCloudRGBPtr cloud, int label_index, bool update) {
    //ROS_INFO("Creating Mats!");
    Mat training_data = Mat(3, 256, CV_32FC1);
    Mat training_label = Mat(1, 1, CV_32FC1); // Just the reponse string in a matrix
    training_label.at<int>(0, 0) = (int) label_index;

    std::vector<uint64_t> histogram = produceColorHist(cloud);

    // Copy histogram contents to testing_data Mat
    memcpy(training_data.data, histogram.data(), sizeof(Mat));

    //ROS_INFO("Training now...");
    bayes->train(training_data, training_label, Mat(), Mat(), update);
    
    return true;
}