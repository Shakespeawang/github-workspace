#ifndef __OPTIMIZATION_FUNCTION__H
#define __OPTIMIZATION_FUNCTION__H


#include <string>
#include <vector>
#include <opencv.hpp>
#include <fstream>

using namespace std;
using namespace cv;


//const static Size board_size = Size(9, 6);
const static string IN_PATH = "D:\\毕业设计\\图漾第二次标定数据\\";
const static string OUT_PATH = "D:\\毕业设计\\图漾第二次标定数据\\out\\";


 bool readParams(std::string path, std::vector<cv::Mat_<float>> & vec1, cv::Size Size1, std::string flag1, std::vector<cv::Mat_<float>> & vec2, cv::Size Size2 = cv::Size(0, 0), std::string flag2 = "", int n = 11);
 bool writeParams(std::string path, std::vector<cv::Mat_<float>> & vec1, std::string flag1, std::vector<cv::Mat_<float>> & vec2, std::string flag2);

 std::vector<cv::Mat> readImage(std::string path, int _k, int _j, int _i = 0, std::string suffix = ".png", std::string connector = "-");
 cv::Mat readImage(int _i, int _j, int _k = 0, std::string path = IN_PATH, std::string suffix = ".png", std::string connector = "-");
 bool writeImage(cv::Mat image, int _i, int _j, int _k = 0, std::string path = OUT_PATH, std::string suffix = ".png", std::string connector = "-");


 void RT2transform(const double R[], const double t[], cv::Mat_<float> & T);
 void transform2RT(const cv::Mat_<float> & T, double R[], double t[]);

 void camera_2_array(const cv::Mat_<float> K, double arr[]);
 void array_2_camera(const double arr[], cv::Mat_<float> & K);

 bool matrix_2_array(const cv::Mat_<float> mat, double arr[], const int arr_row, const int arr_col);
 bool array_2_matrix(cv::Mat_<float> & mat, const double arr[], const int arr_row, const int arr_col);

 vector<Point3d> optimize_array(const std::vector<vector<Point3d>> & arr);
 void optimize_all_observe(
	std::vector <std::vector <std::vector <cv::Point3f > > > & all_observe,
	const cv::vector<cv::Mat_<float>> intrinsic, const cv::vector<cv::Mat_<float>> camera_pos, const cv::vector<cv::Mat_<float>> pose_timestamp,
	const cv::vector<cv::vector<cv::vector<Point3d> > > S_Si, const cv::vector<cv::Point3d> object, const cv::Size board_size
	);
 

 void output_image_point_cloud(
	 const vector < vector < vector < Point3f > > > & all_observe,
	 const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	 const vector<vector<vector<Point3d> > > recon_pts,
	 const string ss1,
	 const bool is_have_si = true
	 );

 bool out_all_point(const vector < vector < vector < Point3f > > > & all_observe, const string ss1 = OUT_PATH + "棋盘格角点.txt");

 bool get_s0_s1(vector<double> & s_0, vector<double> & s_1, vector<Point3f> observe0, vector<Point3f> observe1, Mat K0, Mat K1, Mat camera_pos);


#endif // !__OPTIMIZATION_FUNCTION__H
