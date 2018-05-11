/*
*	@author: liphone
*	@date:	2018/05/08	22:27
*	@two using methods :
*		1. using the function 0 and 8.
*		2. using the function 9 directly.
*/



#ifndef __OPTIMIZATION_FUNCTION__H
#define __OPTIMIZATION_FUNCTION__H

#include <string>
#include <vector>
#include <opencv.hpp>
#include <ceres.h>



typedef struct PairPoint
{
	cv::Point2d imagePoint;
	cv::Point3d worldPoint;

}PairPoint;



/*
*	0)
*	@function :
*		get a pair of points from the plane points to its image points, out the struct of "PairPoint".
*/
void getPairPoints(std::vector<PairPoint> & pairPoints, const std::vector<cv::Point2d> & imagePnts, const std::vector<cv::Point3d> & worldPnts);



/*
*	8)
*	@function :
*		an encapsulation of the process produce of this file.
*/
void estimateOptimize(std::vector<PairPoint> & pairPoints, const cv::Point2d principalPnt, const cv::Size imageSize, cv::Mat & optiK, cv::Mat & optiT, cv::Mat & optiD, cv::Mat & estimateK, cv::Mat & estimateT, cv::Mat & estimateD, const double rate /*0到1为比例, 大于4为数值个数*/, const double f_section = 0/*等于0无上下限*/, const double k0_section = 0/*等于0无上下限*/, const double k1_section = 0 /*等于0无上下限*/);



/*
*	9)
*	@function :
*		a construction of the function estimateOptimize.
*/
void estimateOptimize(const std::vector<cv::Point2d> & imagePnts, const std::vector<cv::Point3d> & worldPnts, const cv::Point2d principalPnt, const cv::Size imageSize, cv::Mat & optiK, cv::Mat & optiT, cv::Mat & optiD, cv::Mat & estimateK, cv::Mat & estimateT, cv::Mat & estimateD, const double rate /*0到1为比例, 大于4为数值个数*/, const double f_section = 0/*等于0无上下限*/, const double k0_section = 0/*等于0无上下限*/, const double k1_section = 0/*等于0无上下限*/);



#endif // !__OPTIMIZATION_FUNCTION__H
