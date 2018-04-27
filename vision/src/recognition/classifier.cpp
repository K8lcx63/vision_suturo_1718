//
// Created by Alex on 20.03.18.
//

#include "classifier.h"
#include "../../../../opencv-3.3.0/modules/core/include/opencv2/core/hal/interface.h"
#include "../../../../opencv-3.3.0/modules/core/include/opencv2/core/mat.hpp"
#include "../../../../opencv-3.3.0/modules/ml/include/opencv2/ml.hpp"

/** std::string labels[2] = {  "CupEcoOrange",
                            "EdekaRedBowl",}; **/


classifier::classifier(){
    bayes = cv::ml::NormalBayesClassifier::create();
}

/**
 * Trains a single PointCloud.
 * @param cloud: A partial view PointCloud of an object
 * @param label_index: The label of this object as an index from mesh_enum in perception.cpp
 * @param update: True keeps previous training data
 * @return Whether the training was successful
 */

bool classifier::train(std::string directory, bool update) {

        if(!update) { // If the classifier is not supposed to be updated, just load the saved classifier data.
            bayes = cv::ml::NormalBayesClassifier::load("normal_bayes_classifier_save");
        }
        else {
            // Iterate through all directories, with one directory for each object
            for (int label_index = 0; label_index < (sizeof(labels) / 8); label_index++) {
                // ../../common_suturo1718/pcd_files / CupEcoOrange
                std::string current_directory = directory + "/" + labels[label_index]; // Directory for this object
                ROS_INFO("Finding the .csv files in the given directory...");
                DIR *dir = opendir(current_directory.c_str());
                struct dirent *ent;
                if (dir) {
                    ROS_INFO("Directory found");
                    while ((ent = readdir(dir)) != NULL) { // Read every .csv one by one
                        if (!has_suffix(ent->d_name, "colors_histogram.csv")) {
                        } else {
                            printf("%s\n", ent->d_name);
                            std::vector<float> parsedCsv;
                            std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file

                            parsedCsv = read_from_file(full_path, parsedCsv); // Put color features into parsedCsv

                            // Now find the correct CVFH-feature .csv
                            const std::string ext("colors_histogram.csv");
                            full_path = full_path.substr(0, full_path.size() -
                                                            ext.size()); // Remove "colors_histogram.csv"
                            full_path = full_path + "normals_histogram.csv"; // Add "normals_histogram.csv"

                            parsedCsv = read_from_file(full_path, parsedCsv); // Add cvfh features to parsedCsv


                            std::vector<float> parsedCsv_normalized;
                            normalize(parsedCsv, parsedCsv_normalized, 1, 0, NORM_L1); // Normalize training data
                            int parsedCsv_total_value_normalized;
                            for(int c = 0; c < parsedCsv_normalized.size(); c++){
                                parsedCsv_total_value_normalized = parsedCsv_total_value_normalized + parsedCsv_normalized[c];
                            }
                            ROS_INFO("Total value after normalizing: %f", parsedCsv_total_value_normalized);


                            for (int parsedCsv_index = 0; parsedCsv_index < parsedCsv_normalized.size(); parsedCsv_index++) {
                                // Fill in sample
                                training_data.at<float>(parsedCsv_index, sample_counter) = parsedCsv_normalized[parsedCsv_index];
                                ROS_INFO("%f", training_data.at<float>(parsedCsv_index, sample_counter));
                            }
                            responses.at<int>(sample_counter) = label_index; // Set label of this sample
                            sample_counter++;
                        }
                    }
                }
            }
            cv::Ptr<cv::ml::TrainData> data = cv::ml::TrainData::create(training_data, cv::ml::ROW_SAMPLE, responses);
            //cv::Ptr<cv::ml::NormalBayesClassifier> classifier = cv::ml::NormalBayesClassifier::create();
            bayes->train(data);
            bayes->save("normal_bayes_classifier_save");
    }
    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classifier::classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features) {
    if(bayes->isTrained()) {
        ROS_INFO("Classifying...");

        cv::Mat predictInput;
        predictInput.create(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
        for(int color_index = 0; color_index < color_features.size(); color_index++){
            predictInput.at<float>(color_index) = color_features[color_index];
            //ROS_INFO("%f", predictInput.at<float>(color_index));
        }
        for(int cvfh_index = 0; cvfh_index < cvfh_features.size(); cvfh_index++){
            predictInput.at<float>(cvfh_index + color_features.size()) = cvfh_features[cvfh_index];
            //ROS_INFO("%f", predictInput.at<float>(cvfh_index + color_features.size()));
        }
        Mat predictInput_normalized;
        normalize(predictInput, predictInput_normalized, 1, 0, NORM_L1);
        for(int xD = 0; xD < color_features.size() + cvfh_features.size(); xD++){
            ROS_INFO("%f", predictInput_normalized.at<float>(xD));
        }
        cv::Mat predictOutputs;
        predictOutputs.create(1, 1, CV_32SC1);
        Mat predictOutputProbs;

        bayes->predictProb(predictInput_normalized, predictOutputs, predictOutputProbs);

        std::cout << predictOutputs << std::endl;
        std::cout << predictOutputProbs << std::endl;
        Mat predictOutputProbsNormalized;
        normalize(predictOutputProbs, predictOutputProbsNormalized, 1, 0, NORM_L1);
        //predictOutputProbsNormalized = normalize_properly(predictOutputProbs);
        std::cout << predictOutputProbsNormalized << std::endl;
    }
    else{
        ROS_ERROR("ERROR: Classifier hasn't been trained, or something went wrong while training!");
    }
    return "xD";
}

/**
 * Returns if given suffix applies to string s
 * @param s
 * @param suffix
 * @return true if suffix is contained, otherwise false
 */
bool classifier::has_suffix(std::string s, std::string suffix) {
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

/**
 * Returns features from a .csv file
 * @param full_path: Path to the file
 * @param parsedCsv: Vector of floats to push back to
 * @return parsedCsv + new floats from file
 */
std::vector<float> classifier::read_from_file(std::string full_path, std::vector<float> parsedCsv){
    std::ifstream data(full_path.c_str());
    if (!data) ROS_INFO("Couldn't open file!");
    else {
        std::string item;
        while (data.is_open()) {
            // Get a new line
            getline(data, item, ',');
            if (!data.eof()) {
                // Remove whitespaces
                for (int i = 0; i < item.length(); i++)
                    if (item[i] == ' ') item.erase(i, 1);
                // Convert string to float
                float item_float = std::strtof(item.c_str(), NULL);
                //ROS_INFO("Item as float: %f", item_float);
                //ROS_INFO("Size of current histogram: %d", parsedCsv.size());
                //ROS_INFO("%f", item_float);
                parsedCsv.push_back(item_float);
            } else {
                ROS_INFO("Finished a file!");
                data.close();
                ROS_INFO("Closed file.");
            }
        }
    }
    return parsedCsv;
}