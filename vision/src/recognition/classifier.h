//
// Created by Alex on 20.03.18.
//

#include <iostream>

#include "../perception/perception.h"
#include "../../../../opencv-3.3.0/modules/ml/include/opencv2/ml.hpp"
#include <dirent.h>

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file
#include <stdio.h>
#include <stdlib.h>

using namespace cv; // OpenCV API is in the C++ "cv" namespace

class classifier {
private:
    Ptr<cv::ml::NormalBayesClassifier> bayes = cv::ml::NormalBayesClassifier::create(); // = cv::ml::NormalBayesClassifier::create();
    int NUMBER_OF_TRAINING_SAMPLES = 2165; // 2165, einzelnd 217
    int ATTRIBUTES_PER_SAMPLE = 308; // 24 + 308
    cv::Mat training_data = cv::Mat(0, ATTRIBUTES_PER_SAMPLE, CV_32FC1); // Input data
    std::vector<int> training_label_vector;
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
public:
    classifier();
    bool train(std::string directory, bool update);
    std::string classify(std::vector<uint64_t> color_features, std::vector<float> cvfh_features);
    bool has_suffix(std::string s, std::string suffix);
    std::vector<float> read_from_file(std::string full_path, std::vector<float> parsedCsv);

};

