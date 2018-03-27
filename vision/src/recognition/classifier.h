//
// Created by Alex on 20.03.18.
//

#include "../perception/perception.h"

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file
#include <stdio.h>

using namespace cv; // OpenCV API is in the C++ "cv" namespace

bool trainAll(std::string directory, bool update);
bool train(PointCloudRGBPtr cloud, int label_index, bool update);
std::string classify(PointCloudRGBPtr cloud);
void batchPCD2histograms(std::string input);