//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

/** std::string labels[2] = {  "CupEcoOrange",
                            "EdekaRedBowl",}; **/


classifier::classifier(){
    random_trees_color_classifier = cv::ml::RTrees::create();
    random_trees_color_classifier->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 50, 0.02));
    random_trees_cvfh_classifier = cv::ml::RTrees::create();
    random_trees_cvfh_classifier->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 50, 0.02));
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
            random_trees_color_classifier = cv::ml::RTrees::load("random_trees_color_save");
            random_trees_cvfh_classifier = cv::ml::RTrees::load("random_trees_cvfh_save");

        }
        else {
            // Iterate through all directories, with one directory for each object
            for (int label_index = 0; label_index < (sizeof(labels) / 8); label_index++) {
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
                            std::vector<float> color_parsedCsv;
                            std::vector<float> cvfh_parsedCsv;

                            std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file

                            color_parsedCsv = read_from_file(full_path, color_parsedCsv); // Put color features into parsedCsv

                            // Now find the correct CVFH-feature .csv
                            const std::string ext("colors_histogram.csv");
                            full_path = full_path.substr(0, full_path.size() -
                                                            ext.size()); // Remove "colors_histogram.csv"
                            full_path = full_path + "normals_histogram.csv"; // Add "normals_histogram.csv"

                            cvfh_parsedCsv = read_from_file(full_path, cvfh_parsedCsv); // Add cvfh features to parsedCsv


                            std::vector<float> color_parsedCsv_normalized;
                            std::vector<float> cvfh_parsedCsv_normalized;

                            normalize(color_parsedCsv, color_parsedCsv_normalized, 1, 0, NORM_L1); // Normalize training data
                            normalize(cvfh_parsedCsv, cvfh_parsedCsv_normalized, 1, 0, NORM_L1); // Normalize training data


                            for (int parsedCsv_index = 0; parsedCsv_index < color_parsedCsv_normalized.size(); parsedCsv_index++) {
                                // Fill in sample
                                color_training_data.at<float>(sample_counter, parsedCsv_index) = color_parsedCsv_normalized[parsedCsv_index];
                                //ROS_INFO("--%f--", training_data.at<float>(sample_counter, parsedCsv_index));
                            }
                            for (int parsedCsv_index = 0; parsedCsv_index < cvfh_parsedCsv_normalized.size(); parsedCsv_index++) {
                                // Fill in sample
                                cvfh_training_data.at<float>(sample_counter, parsedCsv_index) = cvfh_parsedCsv_normalized[parsedCsv_index];
                                //ROS_INFO("--%f--", training_data.at<float>(sample_counter, parsedCsv_index));
                            }

                            responses.at<int>(sample_counter) = label_index; // Set label of this sample
                            sample_counter++;
                        }
                    }
                }
            }
            cv::Ptr<cv::ml::TrainData> color_data = cv::ml::TrainData::create(color_training_data, cv::ml::ROW_SAMPLE, responses);
            cv::Ptr<cv::ml::TrainData> cvfh_data = cv::ml::TrainData::create(cvfh_training_data, cv::ml::ROW_SAMPLE, responses);

            ROS_INFO("Starting to train using the extracted data. This may take a while!");
            random_trees_color_classifier->train(color_data);
            random_trees_cvfh_classifier->train(cvfh_data);

            random_trees_color_classifier->save("random_trees_color_save");
            random_trees_cvfh_classifier->save("random_trees_cvfh_save");

            ROS_INFO("The trained classifiers have been saved."
                             "Setting 'update' to false when starting the node for the next time will cause it to load the data instead of training again!");

    }
    ROS_INFO("%sTraining finished!\n", "\x1B[32m");
    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classifier::classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features) {
    if(random_trees_color_classifier->isTrained() && random_trees_cvfh_classifier->isTrained()) {
        ROS_INFO("Classifying...");

        cv::Mat color_predictInput;
        cv::Mat cvfh_predictInput;

        color_predictInput.create(1, COLOR_ATTRIBUTES_PER_SAMPLE, CV_32FC1);
        cvfh_predictInput.create(1, CVFH_ATTRIBUTES_PER_SAMPLE, CV_32FC1);

        for(int color_index = 0; color_index < color_features.size(); color_index++){
            color_predictInput.at<float>(0, color_index) = color_features[color_index];
            //ROS_INFO("%f", predictInput.at<float>(0, color_index));
        }
        for(int cvfh_index = 0; cvfh_index < cvfh_features.size(); cvfh_index++){
            cvfh_predictInput.at<float>(0, cvfh_index) = cvfh_features[cvfh_index];
            //ROS_INFO("%f", predictInput.at<float>(0, cvfh_index + color_features.size()));
        }
        Mat color_predictInput_normalized;
        Mat cvfh_predictInput_normalized;

        normalize(color_predictInput, color_predictInput_normalized, 1, 0, NORM_L1);
        normalize(cvfh_predictInput, cvfh_predictInput_normalized, 1, 0, NORM_L1);

        for(int xD = 0; xD < color_features.size() + cvfh_features.size(); xD++){
            //ROS_INFO("Feature %d: %f", xD, predictInput_normalized.at<float>(0, xD));
        }

        int color_prediction_result = random_trees_color_classifier->predict(color_predictInput_normalized);
        int cvfh_prediction_result = random_trees_cvfh_classifier->predict(cvfh_predictInput_normalized);

        Mat color_votes;
        Mat cvfh_votes;

        random_trees_color_classifier->getVotes(color_predictInput_normalized, color_votes, 0);
        random_trees_cvfh_classifier->getVotes(cvfh_predictInput_normalized, cvfh_votes, 0);

        Mat combined_votes;
        cv::add(color_votes, cvfh_votes, combined_votes);

        std::cout << color_votes << std::endl;
        std::cout << cvfh_votes << std::endl;
        std::cout << combined_votes << std::endl;


        int color_highest_vote_amount = color_votes.at<int>(1, color_prediction_result);
        int cvfh_highest_vote_amount = cvfh_votes.at<int>(1, cvfh_prediction_result);
        int combined_prediction_result;
        int combined_highest_vote_amount = 0;
        for(int x = 0; x < combined_votes.cols; x++){ // Fill combined_prediction_result and combined_highest_vote_amount manually
            if(combined_highest_vote_amount < combined_votes.at<int>(1, x)){
                combined_prediction_result = x;
                combined_highest_vote_amount = combined_votes.at<int>(1, x);
            }
        }

        int color_vote_percentage = color_highest_vote_amount * 2;
        int cvfh_vote_percentage = cvfh_highest_vote_amount * 2;

        ROS_INFO("Color: %d percent of votes for %s", color_vote_percentage, labels[color_prediction_result].c_str());
        ROS_INFO("CVFH: %d percent of votes for %s", cvfh_vote_percentage, labels[cvfh_prediction_result].c_str());
        ROS_INFO("Combined: %d percent of votes for %s", combined_highest_vote_amount, labels[combined_prediction_result].c_str());


        if(combined_highest_vote_amount > 19){
            ROS_INFO("This is a %s", labels[combined_prediction_result].c_str());
        }
        else{
            ROS_INFO("This is either a %s, or not an object in our dataset.", labels[combined_prediction_result].c_str());
        }

        return labels[combined_prediction_result];
    }
    else{
        ROS_ERROR("ERROR: Classifier hasn't been trained, or something went wrong while training!");
        return "";
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
                //ROS_INFO("%f", item_float);
                parsedCsv.push_back(item_float);
                counter++;
            } else { // In this case, do this once more. eof doesn't mean the file is finished.
                // Remove whitespaces
                for (int i = 0; i < item.length(); i++)
                    if (item[i] == ' ') item.erase(i, 1);
                // Convert string to float
                float item_float = std::strtof(item.c_str(), NULL);
                //ROS_INFO("%f", item_float);
                parsedCsv.push_back(item_float);
                counter++;
                data.close();
                ROS_INFO("Finished and closed a file. Extracted %d features.", counter);
            }
        }
    }
    return parsedCsv;
}