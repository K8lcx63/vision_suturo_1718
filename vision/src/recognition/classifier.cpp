//
// Created by Alex on 20.03.18.
//

#include "classifier.h"

//ROS_INFO("Initializing classifier!");
CvNormalBayesClassifier *bayes = new CvNormalBayesClassifier;


/**
 * Trains all .pcd-files in a directory.
 * @return
 * @param directory: Where the .pcd files are saved
 * @param update: Whether old training data should be kept (true) or deleted (false).
 */
bool trainAll(std::string directory, bool update){
    return true;
}

/**
 * Trains a single PointCloud.
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

/**
 * Classifies a single PointCloud using previously trained data.
 * @param cloud: The cloud to classify
 * @return The label of the classified object
 */

std::string classify(PointCloudRGB cloud){
    return "xd";
}

/**
 * Generates .csv-files from .pcd-files
 * @param input: Directory of the .csv-files
 */

void batchPCD2histograms(std::string input) {
    std::ifstream is(input.c_str());
    std::string line;
    std::string line_trimmed;

    while (getline(is, line)) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>), input_sampler (new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<float> input_cvfhs_features;
        std::vector<uint64_t> input_color_features;

        // load file
        std::cout << "load file " << line << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(line, *input_sampler) != 0){
            //downsample partial view
            std::cout << "input before filtering size is: " << input_sampler->size() << std::endl;

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(input_sampler);
            sor.setLeafSize(0.005f, 0.005f, 0.005f); //from 0.005 (perception) zu
            sor.filter(*input_cloud);

            //prepare files
            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());
            //std::string normals_list = line + "_normals_only.csv";
            //std::ofstream os_normals_list(normals_list.c_str());

            // save empty .csv

            os_normals.close();
            os_colors.close();
            //os_normals_list;

        } else {
            //downsample partial view
            std::cout << "input before filtering size is: " << input_sampler->size() << std::endl;

            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(input_sampler);
            sor.setLeafSize(0.005f, 0.005f, 0.005f); //from 0.005 (perception) zu
            sor.filter(*input_cloud);
            // prepare files
            std::cout << "cloud size is: " << input_cloud->size() << std::endl;
            line.erase(line.size() - 4, 4);
            std::string normals = line + "_normals_histogram.csv";
            std::string colors = line + "_colors_histogram.csv";
            std::ofstream os_normals(normals.c_str());
            std::ofstream os_colors(colors.c_str());
            //std::string normals_list = line + "_curvature.csv";
            //std::ofstream os_normals_list(normals_list.c_str());


            // estimate features

            /*
            std::cout << "estimating cvfh features..." << std::endl;
            input_cvfhs_features = cvfhRecognition(input_cloud); // GetCVFHFeatures instead
            */

            input_color_features = produceColorHist(input_cloud);
            std::cout << "creating color histogram..." << std::endl;

            // save features to .csv
            std::cout << "feature and color histogram size ok. saving file" << std::endl;

            for (int i = 0; i < input_cvfhs_features.size(); i++) {
                if (i == input_cvfhs_features.size() - 1) {
                    os_normals << input_cvfhs_features[i];
                } else {
                    os_normals << input_cvfhs_features[i] << ", ";


                }
            }


            for (int j = 0; j < input_color_features.size(); j++) {
                if (j == input_color_features.size()-1){
                    os_colors << input_color_features[j];

                } else {
                    os_colors << input_color_features[j] << ", ";


                }
            }
            os_normals.close();
            os_colors.close();

        }

    }
}