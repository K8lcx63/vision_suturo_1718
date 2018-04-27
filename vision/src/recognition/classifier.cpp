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
                        //ROS_WARN("This is not a color .csv file");
                    } else {
                        //ROS_INFO("This is a color .csv file");
                        printf("%s\n", ent->d_name);

                        std::vector<float> parsedCsv;
                        // Parse .csv-file
                        std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file

                        //parsedCsv = read_from_file(full_path, parsedCsv); // Put color features into parsedCsv

                        // Now find the correct CVFH-feature .csv
                        const std::string ext("colors_histogram.csv");
                        full_path = full_path.substr(0, full_path.size() - ext.size()); // Remove "colors_histogram.csv"
                        full_path = full_path + "normals_histogram.csv"; // Add "normals_histogram.csv"

                        parsedCsv = read_from_file(full_path, parsedCsv); // Add cvfh features to parsedCsv

                        // Copy histogram contents to testing_data Mat
                        Mat training_data_line = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
                        //memcpy(training_data_line.data, parsedCsv.data(), sizeof(Mat)); // vector to single row Mat
                        for (int copy_counter = 0; copy_counter < ATTRIBUTES_PER_SAMPLE; copy_counter++) {
                            training_data_line.at<float>(copy_counter) = parsedCsv[copy_counter];
                        }
                        Mat training_data_line_normalized = Mat();
                        normalize(training_data_line, training_data_line_normalized); // Normalize training data


                        cv::Size s = training_data_line_normalized.size();
                        ROS_INFO("<----- ----->");
                        for(int xd = 0; xd < s.width; xd++){
                            ROS_INFO("%f", training_data_line_normalized.at<float>(xd));
                        }


                        float label_index_float = (float) label_index;
                        //ROS_INFO("%f", label_index_float);
                        //ROS_INFO("%d", label_index); // TODO: Labels are still correct here, with values from 0-9

                        training_data.push_back(training_data_line_normalized); // Push single row Mat into big Mat
                        // training_label.push_back((int) label_index); // Correctly label this histogram according to input
                        training_label_vector.push_back(label_index);

                    }
                }
            }

            closedir(dir);

        }

        /* // DEBUG: Show error percentage with training set (was 0% last time tested, 24.04.2018)
        bayes = cv::ml::NormalBayesClassifier::load("normal_bayes_classifier_save");
        Mat response;
        Ptr<cv::ml::TrainData> traindata_test = cv::ml::TrainData::create(training_data, cv::ml::ROW_SAMPLE, training_label);
        float error_percentage = bayes->cv::ml::StatModel::calcError(traindata_test, false, response);
        ROS_INFO("Error percentage with training set: %f", error_percentage);
         */

        ROS_INFO("Training now...");
        cv::Size data_size = training_data.size();
        //cv::Size label_size = training_label.size();
        ROS_INFO("data rows: %d", data_size.height);
        ROS_INFO("data columns: %d", data_size.width);
        //ROS_INFO("label rows: %d", label_size.height);
        //ROS_INFO("label columns: %d", label_size.width);
        ROS_INFO("labels total: %d", training_label_vector.size());
        ROS_INFO("amount of different labels: %d", sizeof(labels) / 8);
        if (training_data.data == NULL) {
            ROS_ERROR("TRAINING DATA IS NULL! CAN'T TRAIN!");
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
                // TODO: All labels are 0 here as floats, while being correct as ints!
                Mat test_label_mat = Mat(training_label_vector);
                for(int xDDD = 0; xDDD < training_label_vector.size(); xDDD++) {
                    //ROS_INFO("%f", training_label.at<float>(xDDD));
                    std::cout << training_label_vector[xDDD] << std::endl;
                    std::cout << test_label_mat.at<int>(xDDD) << std::endl;
                }
                 */




                Mat training_label_mat = Mat(0, 0, CV_32SC1);
                for(int xDDD = 0; xDDD < training_label_vector.size(); xDDD++){
                    Mat training_label_mat_single = Mat(0, 0, CV_32SC1);
                    training_label_mat_single.push_back(training_label_vector[xDDD]);
                    training_label_mat.push_back(training_label_mat_single);
                    //training_label_mat.at<int>(xDDD) = training_label_vector[xDDD];
                    //ROS_INFO("%d", training_label_mat.at<int>(xDDD));
                }

                //bayes->train(training_data, training_label, Mat(), Mat(), update);
                //training_label_mat.convertTo(training_label_vector, CV_32F);
                Ptr <cv::ml::TrainData> train_data = cv::ml::TrainData::create(training_data, ml::ROW_SAMPLE,
                                                                               training_label_mat);
                Mat test_training_data = train_data->getSamples();
                Mat test_training_labels = train_data->getResponses();
                ROS_INFO("training data test: %f", test_training_data.at<float>(1, 1));
                cv::Size test_training_labels_size = test_training_labels.size();
                for(int labeltest = 0; labeltest < test_training_labels_size.height; labeltest++) {
                    ROS_INFO("training labels test: %d", test_training_labels.at<int>(labeltest));
                }
                cv::Size label_size = training_label_mat.size();
                ROS_INFO("label rows: %d", label_size.height);
                ROS_INFO("label columns: %d", label_size.width);
                //bayes->train(training_data, ml::ROW_SAMPLE, training_label);
                bayes->train(train_data);
                //bayes->cv::ml::StatModel::train(training_data, 0, training_label);
                ROS_INFO("Finished training!");
            }
        }
        ROS_INFO("Saving...");
        bayes->save("normal_bayes_classifier_save"); // Save the trained classifier
        ROS_INFO("Saved new classifier data!");
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
        std::vector<float> histogram_float;
        /*
        ROS_INFO("COLOR HISTOGRAM");
        for (int f1 = 0; f1 < color_features.size(); f1++) { // Make histogram_float from color features
            float feature_float = color_features[f1]; // Convert int to float
            //ROS_INFO("%f", feature_float);
            histogram_float.push_back(feature_float);
        }
         */


        ROS_INFO("CVFH HISTOGRAM");
        for (int f2 = 0; f2 < cvfh_features.size(); f2++) { // Add cvfh features to same histogram_float
            //ROS_INFO("%f", cvfh_features[f2]);
            histogram_float.push_back(cvfh_features[f2]);
        }

        Mat object_features_mat = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
        //memcpy(object_features_mat.data, histogram_float.data(), sizeof(Mat)); // vector to single row Mat
        for (int copy_counter = 0; copy_counter < ATTRIBUTES_PER_SAMPLE; copy_counter++) { // Copy data into Mat
            object_features_mat.at<float>(copy_counter) = histogram_float[copy_counter];
        }
        Mat object_features_mat_normalized = Mat();
        normalize(object_features_mat, object_features_mat_normalized); // Normalize input Mat

        for(int normalized_counter = 0; normalized_counter < ATTRIBUTES_PER_SAMPLE; normalized_counter++){
            ROS_INFO("%f", object_features_mat_normalized.at<float>(normalized_counter)); // TODO: Returns the same features for every object in a scene?
        }
        /*
        ROS_INFO("----- ----- ----- -----");
        ROS_INFO("%f", object_features_mat.at<float>(0));
        ROS_INFO("%f", object_features_mat.at<float>(1));
        ROS_INFO("%f", object_features_mat.at<float>(2));
        ROS_INFO("-----");
        ROS_INFO("%f", object_features_mat_normalized.at<float>(0));
        ROS_INFO("%f", object_features_mat_normalized.at<float>(1));
        ROS_INFO("%f", object_features_mat_normalized.at<float>(2));
         */

        ROS_INFO("PREDICTING");



        Mat predictprob_result;
        Mat predictprob_result_probabilities;
        int result_int = (int)bayes->predictProb(object_features_mat_normalized, predictprob_result, predictprob_result_probabilities);
        ROS_INFO("Training var count: %d", bayes->getVarCount());
        std::cout << predictprob_result << std::endl;
        std::cout << predictprob_result_probabilities << std::endl;

        ROS_INFO("PREDICTED!");
        ROS_INFO("This is a %d", result_int);
        ROS_INFO("Or could this be a %d", predictprob_result.at<int>(0));
        std::string result_string = labels[result_int]; // Make label string from float
        ROS_INFO("This is a %s", result_string.c_str());
        return result_string;
    }
    else{
        ROS_ERROR("ERROR: Classifier hasn't been trained, or something went wrong while training!");
    }
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