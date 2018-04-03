//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

//ROS_INFO("Initializing classifier!");
CvNormalBayesClassifier *bayes = new CvNormalBayesClassifier;
int NUMBER_OF_TRAINING_SAMPLES = 217;
int ATTRIBUTES_PER_SAMPLE = 768;


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

bool train(std::string directory, int label_index, bool update) {
    //ROS_INFO("Creating Mats!");
    Mat training_data = Mat(NUMBER_OF_TRAINING_SAMPLES, ATTRIBUTES_PER_SAMPLE, CV_32SC1);
    Mat training_label = Mat(NUMBER_OF_TRAINING_SAMPLES, 1, CV_32SC1); // Just the reponse string in a matrix
    //training_label.at<int>(0, 0) = (int) label_index;

    // Find all .csv-files in the given directory
    ROS_INFO("Finding the .csv files in the given directory...");
    DIR *dir = opendir(directory.c_str());
    struct dirent *ent;
    if (dir) {
        ROS_INFO("Directory found");
        while ((ent = readdir(dir)) != NULL) {
            if (!has_suffix(ent->d_name, "colors_histogram.csv")) {
                ROS_WARN("This is not a .csv file");
            } else {
                ROS_INFO("This is a .csv file");
                printf("%s\n", ent->d_name);


                // Parse .csv-file
                ROS_INFO("Parsing .csv file");
                std::string full_path = directory + "/" + ent->d_name;
                std::ifstream data(full_path.c_str()); // Maybe this needs to be the full path, not just the file name?
                std::vector <float> parsedCsv;
                if (!data) ROS_INFO("Couldn't open file!");
                else {
                    std::string item;
                    float item_int;
                    while (data.is_open()) { // TODO: SEGMENTATION FAULT HERE! CHECK IF STREAM IS EMPTY INSTEAD?
                        ROS_INFO("Next entry...");
                        // Get a new line
                        ROS_INFO("Getting line");
                        getline(data, item, ',');
                        if (!data.eof()) {
                            // Remove whitespaces
                            ROS_INFO("Removing commas");
                            for (int i = 0; i < item.length(); i++)
                                if (item[i] == ' ') item.erase(i, 1);
                            ROS_INFO("Converting string to int");
                            // Convert string to int
                            //char** pEnd;
                            float item_float = std::strtof(item.c_str(), NULL);
                            ROS_INFO("Item: %s\n", item.c_str());
                            ROS_INFO("Item as float: %f\n", item_float);
                            ROS_INFO("Pushing back an int");
                            ROS_INFO("Size of current histogram: %d", parsedCsv.size());
                            parsedCsv.push_back(item_int);
                        } else {
                            ROS_INFO("Finished a file!");
                            data.close();
                            ROS_INFO("Closed file.");
                        }
                    }
                }

                ROS_INFO("XD");
                ROS_INFO("Size of current histogram: %d", parsedCsv.size());
                ROS_INFO("Copying histogram contents to data Mat");
                // Copy histogram contents to testing_data Mat
                memcpy(training_data.data, parsedCsv.data(), sizeof(Mat));

                ROS_INFO("Training now...");
                //ROS_INFO("Training now...");
                bayes->train(training_data, training_label, Mat(), Mat(), update);
            }
        }
    }
    closedir(dir);
    return true;
}

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classify(PointCloudRGB cloud) {
    return "xd";
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