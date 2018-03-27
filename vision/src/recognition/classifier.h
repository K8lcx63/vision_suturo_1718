//
// Created by Alex on 20.03.18.
//

#include "../perception/perception.h"

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file
#include <stdio.h>

using namespace cv; // OpenCV API is in the C++ "cv" namespace

// #include "../perception/short_types.h"


bool train(PointCloudRGBPtr cloud);
//std::string classify();
