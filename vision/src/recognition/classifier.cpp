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
 * Trains all .pcd-files in a directory.
 * @return
 * @param directory: Where the .pcd files are saved
 * @param update: Whether old training data should be kept (true) or deleted (false).
 */
bool classifier::train_all(std::string directory, bool update) {
    return true;
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
                        //ROS_WARN("This is not a color .csv file");
                    } else {
                        //ROS_INFO("This is a color .csv file");
                        printf("%s\n", ent->d_name);

                        std::vector<float> parsedCsv;
                        // Parse .csv-file
                        std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file

                        parsedCsv = read_from_file(full_path, parsedCsv);

                        // Now find the correct CVFH-feature .csv
                        const std::string ext("colors_histogram.csv");
                        full_path = full_path.substr(0, full_path.size() - ext.size()); // Remove "colors_histogram.csv"
                        full_path = full_path + "normals_histogram.csv"; // Add "normals_histogram.csv"

                        parsedCsv = read_from_file(full_path, parsedCsv);

                        /*
                        for(int xd = 0; xd < parsedCsv.size(); xd++){
                            ROS_INFO("parsedCsv %f", parsedCsv[xd]);
                        }
                         */

                        ROS_INFO("Copying histogram contents to data Mat");
                        // Copy histogram contents to testing_data Mat
                        Mat training_data_line = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
                        //memcpy(training_data_line.data, parsedCsv.data(), sizeof(Mat)); // vector to single row Mat
                        for (int copy_counter = 0; copy_counter < ATTRIBUTES_PER_SAMPLE; copy_counter++) {
                            training_data_line.at<float>(copy_counter) = parsedCsv[copy_counter];
                        }
                        training_data.push_back(training_data_line); // Push single row Mat into big Mat
                        training_label.push_back(
                                (int) label_index); // Correctly label this histogram according to input
                    }
                }
            }

            closedir(dir);

        }

        ROS_INFO("Training now...");
        cv::Size data_size = training_data.size();
        cv::Size label_size = training_label.size();
        ROS_INFO("data rows: %d", data_size.height);
        ROS_INFO("data columns: %d", data_size.width);
        ROS_INFO("label rows: %d", label_size.height);
        ROS_INFO("label columns: %d", label_size.width);
        ROS_INFO("amount of labels: %d", sizeof(labels) / 8);
        if (training_data.data == NULL || training_label.data == NULL) {
            ROS_ERROR("AT LEAST ONE MAT IS NULL! CAN'T TRAIN!");
        } else {
            if (bayes == NULL) {
                ROS_ERROR("CLASSIFIER IS NULL! CAN'T TRAIN!");
            } else {
                ROS_INFO("Classifier and inputs are fine.");

                /*
                for(int xD = 0; xD < data_size.height; xD++){
                    ROS_INFO("--------------------");
                    for(int xDD = 0; xDD < data_size.width; xDD++){
                        ROS_INFO("%f", training_data.at<float>(xD, xDD));
                    }
                }
                 */

                /*
                for(int xDDD = 0; xDDD < label_size.height; xDDD++){
                    ROS_INFO("%f", training_label.at<float>(xDDD));
                }
                 */

                //bayes->train(training_data, training_label, Mat(), Mat(), update);
                Ptr <cv::ml::TrainData> train_data = cv::ml::TrainData::create(training_data, ml::ROW_SAMPLE,
                                                                               training_label);
                //bayes->train(training_data, ml::ROW_SAMPLE, training_label);
                bayes->train(train_data);
                //bayes->cv::ml::StatModel::train(training_data, 0, training_label);
                ROS_INFO("Finished training!");
                bayes->save("normal_bayes_classifier_save"); // Save the trained classifier
            }
        }
    }
    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classifier::classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features) {
    ROS_INFO("Classifying...");
    std::vector<float> histogram_float;
    ROS_INFO("COLOR HISTOGRAM");
    for(int f1 = 0; f1 < color_features.size(); f1++){ // Make histogram_float from color features
        float feature_float = color_features[f1]; // Convert int to float
        //ROS_INFO("%f", feature_float);
        histogram_float.push_back(feature_float);
    }
    ROS_INFO("CVFH HISTOGRAM");
    for(int f2 = 0; f2 < cvfh_features.size(); f2++){ // Add cvfh features to same histogram_float
        //ROS_INFO("%f", cvfh_features[f2]);
        histogram_float.push_back(cvfh_features[f2]);
    }

    Mat object_features_mat = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
    //memcpy(object_features_mat.data, histogram_float.data(), sizeof(Mat)); // vector to single row Mat
    for(int copy_counter = 0; copy_counter < ATTRIBUTES_PER_SAMPLE; copy_counter++){
        object_features_mat.at<float>(copy_counter) = histogram_float[copy_counter];
    }
    ROS_INFO("PREDICTING");
    float result_float;
    result_float = bayes->predict(object_features_mat);
    ROS_INFO("PREDICTED!");
    ROS_INFO("This is a %f", result_float);
    std::string result_string = labels[static_cast<int>(result_float)]; // Make label string from float
    ROS_INFO("This is a %s", result_string.c_str());
    return result_string;
}

bool classifier::has_suffix(std::string s, std::string suffix) {
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

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