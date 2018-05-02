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
    random_trees_classifier = cv::ml::RTrees::create();
}

/**
 * Trains a single PointCloud.
 * @param cloud: A partial view PointCloud of an object
 * @param label_index: The label of this object as an index from mesh_enum in perception.cpp
 * @param update: True keeps previous training data
 * @return Whether the training was successful
 */

bool classifier::train(std::string directory, bool update) {

    int csv_counter = 0;
        if(!update) { // If the classifier is not supposed to be updated, just load the saved classifier data.
            random_trees_classifier = cv::ml::RTrees::load("random_trees_classifier_save");
        }
        else {
            // Iterate through all directories, with one directory for each object
            for (int label_index = 0; label_index < (sizeof(labels) / 8); label_index++) {
                // ../../common_suturo1718/pcd_files / CupEcoOrange
                std::string current_directory = directory + "/" + labels[label_index]; // Directory for this object
                //std::string current_directory(directory.append(labels[label_index]));
                ROS_INFO("Finding the .csv files in the given directory...");
                DIR *dir = opendir(current_directory.c_str());
                struct dirent *ent;
                if (dir) {
                    ROS_INFO("Directory found");
                    while ((ent = readdir(dir)) != NULL) { // Read every .csv one by one
                        if (!has_suffix(ent->d_name, "colors_histogram.csv")) {
                        } else {
                            csv_counter++;
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
                            normalize(parsedCsv_normalized, parsedCsv_normalized, 1, 0, NORM_L1); // Normalize training data

                            int highest_feature_index = 0;
                            float parsedCsv_total_value_normalized = 0.000000;
                            for (int parsedCsv_index = 0; parsedCsv_index < parsedCsv_normalized.size(); parsedCsv_index++) {
                                // Fill in sample
                                training_data.at<float>(sample_counter, parsedCsv_index) = parsedCsv_normalized[parsedCsv_index];
                                ROS_INFO("--%f--", training_data.at<float>(sample_counter, parsedCsv_index));
                                parsedCsv_total_value_normalized = parsedCsv_total_value_normalized + training_data.at<float>(sample_counter, parsedCsv_index);
                                // Save the highest feature. In case of the feature total not being exactly 1.000000, this one should be corrected.
                                if(training_data.at<float>(sample_counter, parsedCsv_index) > training_data.at<float>(sample_counter, highest_feature_index)){
                                    highest_feature_index = parsedCsv_index;
                                }
                            }
                            ROS_INFO("Total value after normalizing: %f", parsedCsv_total_value_normalized); // Not always exactly 1.000000
                            /*
                            if(parsedCsv_total_value_normalized != 1.000000){ // Correct features in case normalize() messed up
                                training_data.at<float>(sample_counter, highest_feature_index) =
                                        training_data.at<float>(sample_counter, highest_feature_index) - (parsedCsv_total_value_normalized - 1.000000);
                            }

                            parsedCsv_total_value_normalized = 0.000000;
                            for (int parsedCsv_index = 0; parsedCsv_index < parsedCsv_normalized.size(); parsedCsv_index++) {
                                //ROS_INFO("%f", training_data.at<float>(parsedCsv_index, sample_counter));
                                parsedCsv_total_value_normalized = parsedCsv_total_value_normalized + training_data.at<float>(sample_counter, parsedCsv_index);
                            }
                            ROS_INFO("Total value after normalizing AND correcting: %f", parsedCsv_total_value_normalized); // Not always exactly 1.000000
                             */

                            responses.at<int>(sample_counter) = label_index; // Set label of this sample
                            sample_counter++;
                        }
                    }
                }
            }
            ROS_INFO("There were %d color histogram CSVs!", csv_counter);
            cv::Ptr<cv::ml::TrainData> data = cv::ml::TrainData::create(training_data, cv::ml::ROW_SAMPLE, responses);
            ROS_INFO("Starting to train using the extracted data. This may take a while!");
            random_trees_classifier->train(data);
            random_trees_classifier->save("random_trees_classifier_save");
            ROS_INFO("The trained classifier has been saved as 'normal_bayes_classifier_save'."
                             "Setting 'update' to false when starting the node for the next time will cause it to load the data instead of training again!");
            /*
            int test_result = -1;
            test_result = classifier->predict(training_data.row(2));
            ROS_INFO("Sample 300 is a %d", test_result);
            */
            Mat testing_data;
            training_data.row(1500).copyTo(testing_data);
            // Below that ,it's similar to 36, with some samples normalized sums being closer to 1.
            // 36: Look somewhat valid. Doesn't add up to 1. [2.31393e+36, 0] -> [1, 0]
            // 37-38: Look somewhat valid. Don't add up to 1. [inf, 0] -> [-nan, 0]
            // 39: Looks somewhat valid, but three are 0. Doesn't add up to 1. [inf, 0] -> [-nan, 0]
            // 40: Single digit looks valid, others 0. [inf, inf] -> [-nan, -nan]
            // Above that, it's usually all zeroes. [inf, inf] -> [-nan, -nan]
            //std::cout << testing_data << std::endl;


            testing_data.at<float>(1) = testing_data.at<float>(1) + 0.000050;
            testing_data.at<float>(10) = testing_data.at<float>(10) - 0.000050;

            testing_data.at<float>(2) = testing_data.at<float>(2) + 0.000100;
            testing_data.at<float>(9) = testing_data.at<float>(9) - 0.000100;

             /*
            ROS_INFO("%f", testing_data.at<float>(10));
            ROS_INFO("%f", testing_data.at<float>(1));
             */
            int test_result_2 = -1;
            test_result_2 = random_trees_classifier->predict(testing_data);
            ROS_INFO("Slightly modified sample 1500 is a %d", test_result_2);


            /*
            cv::Mat predictOutputs;
            predictOutputs.create(1, 1, CV_32SC1);
            Mat predictOutputProbs;
            predictOutputProbs.create(1, 10, CV_32FC1);

            classifier->predict(testing_data, predictOutputs, predictOutputProbs);

            std::cout << predictOutputs << std::endl;
            std::cout << predictOutputProbs << std::endl;
            Mat predictOutputProbsNormalized;
            normalize(predictOutputProbs, predictOutputProbsNormalized, 1, 0, NORM_L1);
            std::cout << predictOutputProbsNormalized << std::endl;
             */

    }
    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classifier::classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features) {
    if(random_trees_classifier->isTrained()) {
        ROS_INFO("Classifying...");

        cv::Mat predictInput;
        predictInput.create(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
        for(int color_index = 0; color_index < color_features.size(); color_index++){
            predictInput.at<float>(0, color_index) = color_features[color_index];
            //ROS_INFO("%f", predictInput.at<float>(0, color_index));
        }
        for(int cvfh_index = 0; cvfh_index < cvfh_features.size(); cvfh_index++){
            predictInput.at<float>(0, cvfh_index + color_features.size()) = cvfh_features[cvfh_index];
            //ROS_INFO("%f", predictInput.at<float>(0, cvfh_index + color_features.size()));
        }
        Mat predictInput_normalized;
        normalize(predictInput, predictInput_normalized, 1, 0, NORM_L1);
        int highest_feature_index = 0;
        float total_features_normalized = 0.000000;
        for(int xD = 0; xD < color_features.size() + cvfh_features.size(); xD++){
            ROS_INFO("Feature %d: %f", xD, predictInput_normalized.at<float>(0, xD));
            total_features_normalized = total_features_normalized + predictInput_normalized.at<float>(xD);
            if(predictInput_normalized.at<float>(xD) > predictInput_normalized.at<float>(highest_feature_index)){
                highest_feature_index = xD;
            }
        }

        ROS_INFO("Total: %f", total_features_normalized);
        /*
        if(total_features_normalized != 1.000000){ // Correct features in case normalize() messed up
            ROS_INFO("corrected by %f", (total_features_normalized - 1.000000));
            predictInput_normalized.at<float>(highest_feature_index) =
                    predictInput_normalized.at<float>(highest_feature_index) - (total_features_normalized - 1.000000);
        }
         */




        cv::Mat predictOutputs;
        predictOutputs.create(1, 1, CV_32SC1);
        Mat predictOutputProbs;
        predictOutputProbs.create(1, 10, CV_32FC1);

        int prediction_result = random_trees_classifier->predict(predictInput_normalized);

        ROS_INFO("This is a %d", prediction_result);
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
    int counter = 0;
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
                counter++;
            } else { // In this case, do this once more. eof doesn't mean the file is finished.
                // Remove whitespaces
                for (int i = 0; i < item.length(); i++)
                    if (item[i] == ' ') item.erase(i, 1);
                // Convert string to float
                float item_float = std::strtof(item.c_str(), NULL);
                //ROS_INFO("Item as float: %f", item_float);
                //ROS_INFO("Size of current histogram: %d", parsedCsv.size());
                //ROS_INFO("%f", item_float);
                parsedCsv.push_back(item_float);
                counter++;

                ROS_INFO("Finished a file!");
                data.close();
                ROS_INFO("Closed file. Extracted %d features.", counter);
            }
        }
    }
    return parsedCsv;
}