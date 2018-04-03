//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

std::string labels[10] = {  "CupEcoOrange",
                            "EdekaRedBowl",
                            "HelaCurryKetchup",
                            "JaMilch",
                            "KellogsToppasMini",
                            "KoellnMuesliKnusperHonigNuss",
                            "PringlesPaprika",
                            "PringlesSalt",
                            "SiggBottle",
                            "TomatoSauceOroDiParma"};

//ROS_INFO("Initializing classifier!");
CvNormalBayesClassifier *bayes = new CvNormalBayesClassifier;
int NUMBER_OF_TRAINING_SAMPLES = 217;
int ATTRIBUTES_PER_SAMPLE = 332; // 24 + 308


/**
 * Trains all .pcd-files in a directory.
 * @return
 * @param directory: Where the .pcd files are saved
 * @param update: Whether old training data should be kept (true) or deleted (false).
 */
bool train_all(std::string directory, bool update) {
    return true;
}

/**
 * Trains a single PointCloud.
 * @param cloud: A partial view PointCloud of an object
 * @param label_index: The label of this object as an index from mesh_enum in perception.cpp
 * @param update: True keeps previous training data
 * @return Whether the training was successful
 */

bool train(std::string directory, bool update) {
    Mat training_data = Mat(0, ATTRIBUTES_PER_SAMPLE, CV_32FC1); // Input data
    Mat training_label = Mat(0, 1, CV_32SC1); // Output labels
    bool color_or_cvfh = false;
    std::vector<float> parsedCsv;

    // Iterate through all directories, with one directory for each object
    for(int label_index = 0; label_index < (sizeof(labels) / 8); label_index++) {
        std::string current_directory = directory + "/" + labels[label_index]; // Directory for this object
        ROS_INFO("Finding the .csv files in the given directory...");
        DIR *dir = opendir(current_directory.c_str());
        struct dirent *ent;
        if (dir) {
            ROS_INFO("Directory found");
            while ((ent = readdir(dir)) != NULL) { // Read every .csv one by one
                if (!has_suffix(ent->d_name, "colors_histogram.csv") && !has_suffix(ent->d_name, "normals_histogram.csv")) {
                    ROS_WARN("This is not a .csv file");
                } else {
                    ROS_INFO("This is a .csv file");
                    // TODO: These don't seem to go one after another 100% of the time. Check for specific names instead?
                    color_or_cvfh = !color_or_cvfh; // If last one was color, this is cvfh. As well as other way around.
                    printf("%s\n", ent->d_name);


                    // Parse .csv-file
                    std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file
                    std::ifstream data(full_path.c_str());
                    if(color_or_cvfh){
                        parsedCsv.clear(); // If this is a color .csv, clear parsedCsv, since this is a new object.
                    }
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
                    if(!color_or_cvfh) { // Only do this if this is the second file for this object
                        ROS_INFO("Copying histogram contents to data Mat");
                        // Copy histogram contents to testing_data Mat
                        Mat training_data_line = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
                        memcpy(training_data_line.data, parsedCsv.data(), sizeof(Mat)); // vector to single row Mat
                        training_data.push_back(training_data_line); // Push single row Mat into big Mat
                        training_label.push_back(label_index); // Correctly label this histogram according to input
                    }
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
    ROS_INFO("amount of labels: %d", sizeof(labels));
    bayes->train(training_data, training_label, Mat(), Mat(), update);

    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features) {
    ROS_INFO("Classifying...");
    std::vector<float> histogram_float;
    for(int f1 = 0; f1 < color_features.size(); f1++){ // Make histogram_float from color features
        ROS_INFO("%d", color_features[f1]);
        histogram_float.push_back(color_features[f1]);
    }
    for(int f2 = 0; f2 < cvfh_features.size(); f2++){ // Add cvfh features to same histogram_float
        ROS_INFO("%f", cvfh_features[f2]);
        histogram_float.push_back(cvfh_features[f2]);
    }

    Mat object_features_mat = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
    memcpy(object_features_mat.data, histogram_float.data(), sizeof(Mat)); // vector to single row Mat
    ROS_INFO("PREDICTING");
    float result_float = bayes->predict(object_features_mat);

    std::string result_string = labels[static_cast<int>(result_float)]; // Make label string from float
    ROS_INFO("This is a %s", result_string.c_str());
    return result_string;
}

bool has_suffix(const string &s, const string &suffix) {
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}