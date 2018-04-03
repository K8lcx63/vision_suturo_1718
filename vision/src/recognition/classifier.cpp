//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

std::string labels[] = {"CupEcoOrange",
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
//int ATTRIBUTES_PER_SAMPLE = 768;
//int NUMBER_OF_TRAINING_SAMPLES = 1;
int ATTRIBUTES_PER_SAMPLE = 24; // Should be this, but throws error
//int ATTRIBUTES_PER_SAMPLE = 300; // Doesn't make sense, but trains properly, taking very long


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

    // Iterate through all directories, with one directory for each object
    for(int label_index = 0; label_index < sizeof(labels); label_index++) {
        std::string current_directory = directory + "/" + labels[label_index]; // Directory for this object
        ROS_INFO("Finding the .csv files in the given directory...");
        DIR *dir = opendir(current_directory.c_str());
        struct dirent *ent;
        if (dir) {
            ROS_INFO("Directory found");
            while ((ent = readdir(dir)) != NULL) { // Read every .csv one by one
                if (!has_suffix(ent->d_name, "colors_histogram.csv")) {
                    ROS_WARN("This is not a .csv file");
                } else {
                    ROS_INFO("This is a .csv file");
                    printf("%s\n", ent->d_name);


                    // Parse .csv-file
                    std::string full_path = current_directory + "/" + ent->d_name; // Path of this .csv file
                    std::ifstream data(full_path.c_str());
                    std::vector<float> parsedCsv;
                    //Mat parsedCsv = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
                    if (!data) ROS_INFO("Couldn't open file!");
                    else {
                        std::string item;
                        while (data.is_open()) { // TODO: SEGMENTATION FAULT HERE! CHECK IF STREAM IS EMPTY INSTEAD?
                            // Get a new line
                            getline(data, item, ',');
                            if (!data.eof()) {
                                // Remove whitespaces
                                for (int i = 0; i < item.length(); i++)
                                    if (item[i] == ' ') item.erase(i, 1);
                                // Convert string to float
                                float item_float = std::strtof(item.c_str(), NULL);
                                ROS_INFO("Item as float: %f", item_float);
                                //ROS_INFO("Size of current histogram: %d", parsedCsv.size());
                                parsedCsv.push_back(item_float);
                            } else {
                                ROS_INFO("Finished a file!");
                                data.close();
                                ROS_INFO("Closed file.");
                            }
                        }
                    }

                    ROS_INFO("Copying histogram contents to data Mat");
                    // Copy histogram contents to testing_data Mat
                    Mat training_data_line = Mat(1, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
                    memcpy(training_data_line.data, parsedCsv.data(), sizeof(Mat)); // vector to single row Mat
                    training_data.push_back(training_data_line); // Push single row Mat into big Mat
                    training_label.push_back(label_index); // Correctly label this histogram according to input
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

std::string classify(std::vector<uint64_t> histogram) {
    ROS_INFO("Classifying...");
    Mat m = Mat();
    m.push_back(histogram);
    Mat result = Mat();
    float result_float = bayes->predict(m);
    ROS_INFO("This is a %f", result_float);

    std::string result_string = labels[static_cast<int>(result_float)]; // Make label string from float
    return result_string;
}

int read_data_from_csv(const char *filename, Mat data, Mat classes, int n_samples) {
    char tmpc;
    float tmpf;

    // if we can't read the input file then return 0
    FILE *f = fopen(filename, "r");
    if (!f) {
        printf("ERROR: cannot read file %s\n", filename);
        return 0; // all not OK
    }

    // for each sample in the file

    for (int line = 0; line < n_samples; line++) {

        // for each attribute on the line in the file

        for (int attribute = 0; attribute < (ATTRIBUTES_PER_SAMPLE + 2); attribute++) {
            if (attribute == 0) {
                fscanf(f, "%f,", &tmpf);

                // ignore attribute 0 (as it's the patient ID)

                continue;
            } else if (attribute == 1) {

                // attribute 2 (in the database) is the classification
                // record 1 = M = malignant
                // record 0 = B = benign

                fscanf(f, "%c,", &tmpc);

                switch (tmpc) {
                    case 'M':
                        classes.at<float>(line, 0) = 1.0;
                        break;
                    case 'B':
                        classes.at<float>(line, 0) = 0.0;
                        break;
                    default:
                        printf("ERROR: unexpected class in file %s\n", filename);
                        return 0; // all not OK
                }

                // printf("%c,", tmpc);
            } else {
                fscanf(f, "%f,", &tmpf);
                data.at<float>(line, (attribute - 2)) = tmpf;
                //printf("%f,", tmpf);
            }
        }
        fscanf(f, "\n");
        //printf("\n");
    }

    fclose(f);

    return 1; // all OK
}

bool has_suffix(const string &s, const string &suffix) {
    return (s.size() >= suffix.size()) && equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}