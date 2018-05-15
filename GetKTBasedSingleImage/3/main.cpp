#include <opencv.hpp>
#include <ceres.h>
#include "optimize.h"


using namespace cv;
using namespace std;


cv::Mat readAImage(int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	cv::Mat image = imread(filename.str());
	return image;
}

bool readParams(std::vector<cv::Mat> & v, const std::string path, size_t nums, cv::Size size, std::string tag)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < nums; i++)
	{
		std::stringstream ss;
		ss << tag << i;
		cv::Mat m(size, CV_64FC1);
		fs[ss.str()] >> m;
		assert(!m.empty());
		v.push_back(m);
	}
	fs.release();
	return true;
}


int main(int argc,char * argv[])
{
	string image_path = "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\matlab\\";
	string Ki_xml = "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\matlab\\K.yaml";
	string Ti_xml = "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\matlab\\T.yaml";
	int camera_nums = 1, image_nums = 4;
	Size boardSize(12,13), squareSize(30, 30);

	//读入相机的内外参数
	vector<Mat>camera_pos, pose_timestamp, distortion, intrinsic;
	readParams(intrinsic, Ki_xml, camera_nums, Size(3, 3), "K_");
	readParams(distortion, Ki_xml, camera_nums, Size(1, 5), "DISTORTION_");
	readParams(camera_pos, Ti_xml, camera_nums, Size(4, 4), "camera_pose_");
	readParams(pose_timestamp, Ti_xml, image_nums, Size(4, 4), "pose_timestamp_");

	double f_bound[2] = { 0 }, k1_bound[2] = { 0 }, k2_bound[2] = { 0 };
	for (size_t i = 0; i < camera_nums; i++)
	{
		for (size_t j = 0; j < image_nums; j++)
		{
			Mat realImg = readAImage(0, i, j, image_path, ".jpg", "-");
			
			Point2d principalPt(intrinsic[i].at<double>(0, 2), intrinsic[i].at<double>(1, 2));
			double focalLength;
			Mat rotMatrix, tranVect, dist;
			imageRun(realImg, boardSize, squareSize, principalPt, focalLength, rotMatrix, tranVect, dist, f_bound, k1_bound, k2_bound);

			cout << "True value:" << endl;
			cout << "K:" << endl << intrinsic[i] << endl;
			cout << "T:" << endl << pose_timestamp[j] << endl;
			cout << "D:" << endl << distortion[i] << endl;

			cout << "Optimized value:" << endl;
			cout << "focalLength:" << focalLength << endl;
			cout << "rotMatrix:" << endl << rotMatrix << endl;
			cout << "tranVect:" << endl << tranVect << endl;
			cout << "dist:" << endl << dist << endl << endl << endl;
		}
	}
	system("pause");
	return	0;
}




