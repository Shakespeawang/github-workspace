
#ifndef __BINOCULAR_CALIBRATION__H
#define __BINOCULAR_CALIBRATION__H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
//#include "extra.h" // use this if in OpenCV2 
#include <ceres/ceres.h>

void pose_estimate(const cv::Mat& img_1, const cv::Mat& img_2, const cv::Mat& K, cv::Mat& R, cv::Mat& t);

#endif // !__BINOCULAR_CALIBRATION__H

