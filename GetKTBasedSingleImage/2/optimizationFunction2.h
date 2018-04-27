#ifndef __OPTIMIZATION_FUNCTION__H
#define __OPTIMIZATION_FUNCTION__H


#include <string>
#include <vector>
#include <opencv.hpp>

using namespace std;
using namespace cv;

void RT2transform(const double R[], const double t[], cv::Mat & T);
void transform2RT(const cv::Mat & T, double R[], double t[]);

void camera_2_array(const cv::Mat & K, double arr[]);
void array_2_camera(const double arr[], cv::Mat & K);

bool matrix_2_array(const cv::Mat mat, double arr[], const int arr_row, const int arr_col);
bool array_2_matrix(cv::Mat & mat, const double arr[], const int arr_row, const int arr_col);

bool readParams(std::vector<cv::Mat> & v, const std::string path, size_t nums, cv::Size size, std::string tag);


vector<Mat> readImage(string path, int _i, int _j, int _k, string suffix, string connector);
Mat readAImage(int _i, int _j, int _k, string path, string suffix, string connector);

cv::Size getCornerPoints(
	vector < vector < vector < Point2d > > > & all_observe,
	const Size board_size, const string image_path, const size_t camera_num, const size_t image_num);


#endif // !__OPTIMIZATION_FUNCTION__H
