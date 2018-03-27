//
// Created by Alex on 20.03.18.
//


#include "../perception/perception.h"
#include <dirent.h>
#include <boost/lexical_cast.hpp>

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file
#include <stdio.h>

using namespace cv; // OpenCV API is in the C++ "cv" namespace

bool train_all(std::string directory, bool update);
bool train(std::string directory, int label_index, bool update);
std::string classify(PointCloudRGBPtr cloud);
int read_data_from_csv(const char* filename, Mat data, Mat classes, int n_samples);
bool has_suffix(const string& s, const string& suffix);