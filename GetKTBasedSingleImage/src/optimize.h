


#ifndef __OPTIMIZATION_FUNCTION__H
#define __OPTIMIZATION_FUNCTION__H

#include <string>
#include <vector>
#include <opencv.hpp>
#include <ceres.h>



/*
*	@function :
*/
bool imageRun(const cv::Mat & realImg, const cv::Size boardSize, const cv::Size squareSize, const cv::Point2d principalPt, double & focalLength, cv::Mat & rotMatrix, cv::Mat & tranVect, cv::Mat & dist, double f_bound[2], double k1_bound[2], double k2_bound[2]);


#endif // !__OPTIMIZATION_FUNCTION__H
