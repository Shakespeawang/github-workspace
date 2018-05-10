/*
*	@author: liphone
*	@date:	2018/05/08	22:27
*	@brief introduction :
*		A example of using the api of "optimize.h".
*/
#include <opencv.hpp>
#include <ceres.h>
#include "optimize.h"

using namespace cv;
using namespace std;


bool readParams(std::vector<cv::Mat> & v, const std::string path, size_t nums, cv::Size size, std::string tag);
vector<Mat> readImage(string path, int _i, int _j, int _k, string suffix, string connector);
Mat readAImage(int _i, int _j, int _k, string path, string suffix, string connector);
cv::Size getCornerPoints(	vector < vector < vector < Point2d > > > & all_observe,	const Size board_size, const string image_path, const size_t camera_num, const size_t image_num);


void randomMakeObjectPoints(vector<PairPoint> & pairPoints, const Rect rect, const int Z, const size_t nums);
void makeImagePoints(vector<PairPoint> & pairPoints, const Mat K, const Mat T, const Mat D, const Size imageSize, const bool isHaveDistortion);


void help(){
	std::cout << "CalibrationOptimization -img image_path camera_nums image_nums -b 9 6 -s 50 50 -i Ki.xml Ti.xml" << endl;
	std::cout << "\t-image 'image path' 'camera nums' 'image nums'" << endl;
	std::cout << "\t-board-size 'board width' 'board height'" << endl;
	std::cout << "\t-square-size 'square width' 'square height'" << endl;
	std::cout << "\t-input Ki.xml Ti.xml" << endl;
}


void monitorTest()
{
	Mat K = (Mat_<double>(3, 3) <<
		1000, 0, 400,
		0, 800, 300,
		0, 0, 1);
	Mat D = (Mat_<double>(1, 5) << 0.01, 0.02, -0.000003, -0.000004, 0);
	/*Mat T1 = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
		);*/
	Mat T1 = (Mat_<double>(3, 3) <<
		8.12486336e-002, - 7.45385349e-001, 6.61664069e-001, 
		9.94988263e-001, 9.94789228e-002, - 1.01128444e-002,
		- 5.82836643e-002, 6.59169674e-001, 7.49732196e-001
		);

	//double t[3] = { 0, 0, 0 };
	double t[3] = { 1.42059540e+002, -6.71977615e+001, 1.40762988e+003 };
	Mat T = (Mat_<double>(3, 4) <<
		T1.at<double>(0, 0), T1.at<double>(0, 1), T1.at<double>(0, 2), t[0],
		T1.at<double>(1, 0), T1.at<double>(1, 1), T1.at<double>(1, 2), t[1],
		T1.at<double>(2, 0), T1.at<double>(2, 1), T1.at<double>(2, 2), t[2]
		);

	Point2d principalPnt(K.at<double>(0, 2), K.at<double>(1, 2));
	vector<PairPoint> pairPoints;
	randomMakeObjectPoints(pairPoints, Rect(0, 0, 1000, 1000), 1.0, 1000);//在2000mm * 2000mm * 200mm立方体内随机生成空间点
	makeImagePoints(pairPoints, K, T, D, Size(principalPnt.x * 2, principalPnt.y * 2), true);

	Mat optiK, optiT, optiD, estimateK, estimateT, estimateD;

	estimateOptimize(pairPoints, principalPnt, Size(principalPnt.x * 2, principalPnt.y * 2), optiK, optiT, optiD, estimateK, estimateT, estimateD, 8);

	cout << "模拟环境测试：" << endl;
	cout << "True value:" << endl;
	cout << "K:" << endl << K << endl;
	cout << "T:" << endl << T << endl;
	cout << "D:" << endl << D << endl;

	cout << "Estimate value:" << endl;
	cout << "K:" << endl << estimateK << endl;
	cout << "T:" << endl << estimateT << endl;
	cout << "D:" << endl << estimateD << endl;

	cout << "Optimized Estimate value:" << endl;
	cout << "K:" << endl << optiK << endl;
	cout << "T:" << endl << optiT << endl;
	cout << "D:" << endl << optiD << endl;
}

int main(int argfc,char * arfgv[])
{
	int argc = 14;
	char * argv[] = {
	"CO",
	"-img", "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\apple6s\\", "1", "3",
	"-b", "9", "7",
	"-s", "20", "20",
	"-i", "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\apple6s\\K.yaml", "D:\\毕业设计\\工程项目\\CalibrationOptimization\\Release\\apple6s\\T.yaml"};
	
	try {
		if (argc < 2){
			help(); return 0;
		}
		monitorTest();

		string image_path, Ki_xml, Ti_xml;
		int camera_nums, image_nums;
		Size board_size, square_size;
		for (int i = 1; i < argc;) {
			if (!strcmp(argv[i], "-img")) {
				image_path = argv[++i];
				camera_nums = atoi(argv[++i]);
				image_nums = atoi(argv[++i]);
				++i;
			}
			else if (!strcmp(argv[i], "-b")) {
				board_size.width = atoi(argv[++i]);
				board_size.height = atoi(argv[++i]);
				++i;
			}
			else if (!strcmp(argv[i], "-s")) {
				square_size.width = atoi(argv[++i]);
				square_size.height = atoi(argv[++i]);
				++i;
			}
			else if (!strcmp(argv[i], "-i")) {
				Ki_xml = argv[++i];
				Ti_xml = argv[++i];
				++i;
			}
		}

		//读入相机的内外参数
		vector<Mat>camera_pos, pose_timestamp, distortion, intrinsic;
		readParams(intrinsic, Ki_xml, camera_nums, Size(3, 3), "K_");
		readParams(distortion, Ki_xml, camera_nums, Size(1, 5), "DISTORTION_");
		readParams(camera_pos, Ti_xml, camera_nums, Size(4, 4), "camera_pose_");
		readParams(pose_timestamp, Ti_xml, image_nums, Size(4, 4), "pose_timestamp_");

		//读入棋盘格所有的角点
		vector < vector < vector < Point2d > > > all_observe;
		Size imageSize = getCornerPoints(all_observe, board_size,image_path,camera_nums,image_nums);

		vector<Point3d> object;		object.clear();
		for (size_t j = 0; j < board_size.height; j++)
		{
			for (size_t k = 0; k < board_size.width; k++)
			{
				Point3f pnt;
				pnt.x = k * square_size.width;
				pnt.y = j * square_size.height;
				pnt.z = 0;
				object.push_back(pnt);
			}
		}

		for (size_t i = 0; i < all_observe.size(); i++)
		{
			for (size_t j = 0; j < all_observe[i].size(); j++)
			{
				Point2d principalPnt(intrinsic[i].at<double>(0, 2), intrinsic[i].at<double>(1, 2));

				Mat optiK, optiT, optiD, estimateK, estimateT, estimateD;
				estimateOptimize(all_observe[i][j], object, principalPnt, imageSize, optiK, optiT, optiD, estimateK, estimateT, estimateD);

				cout << "真实环境测试：" << endl;
				cout << "True value:" << endl;
				cout << "K:" << endl << intrinsic[i] << endl;
				cout << "T:" << endl << pose_timestamp[j] << endl;
				cout << "D:" << endl << distortion[i] << endl;

				cout << "Estimate value:" << endl;
				cout << "K:" << endl << estimateK << endl;
				cout << "T:" << endl << estimateT << endl;
				cout << "D:" << endl << estimateD << endl;

				cout << "Optimized Estimate value:" << endl;
				cout << "K:" << endl << optiK << endl;
				cout << "T:" << endl << optiT << endl;
				cout << "D:" << endl << optiD << endl;
			}
		}
	}
	catch (exception e){
		cout << e.what() << endl;
	}
	system("pause");
	return	0;
}


void randomMakeObjectPoints(vector<PairPoint> & pairPoints, const Rect rect, const int Z, const size_t nums)
{
	for (size_t i = 0; i < nums; i++)
	{
		PairPoint pt;
		pt.worldPoint.x = rand() % (rect.width) + rand()*1.0 / RAND_MAX - rect.width / 2.0;
		pt.worldPoint.y = rand() % (rect.height) + rand()*1.0 / RAND_MAX - rect.height / 2.0;
		pt.worldPoint.z = 0;// rand() % Z + rand()*1.0 / RAND_MAX - Z / 2.0;
		//以立方体中心为原点，随机生成一系列空间点
		pairPoints.push_back(pt);
	}
}



void makeImagePoints(vector<PairPoint> & pairPoints, const cv::Mat K, const cv::Mat T, const cv::Mat D, const Size imageSize, const bool isHaveDistortion)
{
	//投影
	for (size_t i = 0; i < pairPoints.size();)
	{
		cv::Mat W1 = (cv::Mat_<double>(4, 1) << pairPoints[i].worldPoint.x, pairPoints[i].worldPoint.y, pairPoints[i].worldPoint.z, 1.0);

		Mat W2 = T * W1;

		double fx = K.at<double>(0, 0);
		double cx = K.at<double>(0, 2);
		double fy = K.at<double>(1, 1);
		double cy = K.at<double>(1, 2);

		double x = W2.at<double>(0, 0) / W2.at<double>(2, 0);
		double y = W2.at<double>(1, 0) / W2.at<double>(2, 0);

		double x_distorted = x;
		double y_distorted = y;
		if (isHaveDistortion){
			double k1 = D.at<double>(0, 0);
			double k2 = D.at<double>(0, 1);
			double p1 = D.at<double>(0, 2);
			double p2 = D.at<double>(0, 3);
			double k3 = D.at<double>(0, 4);

			double r2 = x * x + y * y;

			double d_k = (1.0) + k1 * r2 + k2 * pow(r2, 2);// +k3 * pow(r2, 3);

			x_distorted = x * d_k + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
			y_distorted = y * d_k + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);
		}

		Point2d pts;
		pts.x = fx * x_distorted + cx;
		pts.y = fy * y_distorted + cy;

		if (pts.x >= 0 && pts.x < imageSize.width && pts.y >= 0 && pts.y < imageSize.height){
			pairPoints[i].imagePoint = pts;
			i++;
		}
		else
			pairPoints.erase(pairPoints.begin() + i);

		//double fx = K.at<double>(0, 0);
		//double cx = K.at<double>(0, 2);
		//double fy = K.at<double>(1, 1);
		//double cy = K.at<double>(1, 2);

		
	}
}



Size getCornerPoints(
	vector < vector < vector < Point2d > > > & all_observe,
	Size board_size, string image_path, size_t camera_num, size_t image_num)
{
	cv::Size imageSize;
	for (int i = 0; i < camera_num; i++)
	{
		vector<Mat> vec_image_i = readImage(image_path, 0, camera_num - 1, image_num, ".tif", "-");

		vector<vector<Point2d>> observe_s_j; observe_s_j.clear();

		for (int j = 0; j < image_num; j++) {
			if (0 == j)
				imageSize = vec_image_i[0].size();

			vector<Point2f> observe;
			vector<Point2d> observe2d;

			cv::Mat result_img;
			vec_image_i[j].copyTo(result_img);

			bool found = findChessboardCorners(result_img, board_size, observe, CALIB_CB_ADAPTIVE_THRESH);
			if (!found) {
				observe.clear();
			}
			else{
				cv::Mat view_gray;
				cvtColor(result_img, view_gray, CV_RGB2GRAY);
				find4QuadCornerSubpix(view_gray, observe, Size(5, 5));
			}
			for (size_t k = 0; k < observe.size(); k++)
			{
				observe2d.push_back(Point2d(observe[k].x, observe[k].y));
			}
			observe_s_j.push_back(observe2d);
#ifdef _DEBUG
			cv::Mat img0;
			//vec_image_i[_j].copyTo(img0);
			//cv::cvtColor(vec_image0[_j], img0, CV_GRAY2RGB);
			cv::drawChessboardCorners(vec_image_i[j], board_size, observe, found);
			img0 = vec_image_i[j];
#endif // _DEBUG

		}
		all_observe.push_back(observe_s_j);
	}
	return imageSize;
}



vector<Mat> readImage(string path, int _i, int _j, int _k, string suffix, string connector)
{
	vector<Mat> vec;
	vec.clear();
	for (int i = _i; i <= _i; i++) {
		for (int j = _j; j <= _j; j++) {
			for (int k = 0; k < _k; k++) {
				stringstream ss;
				ss << path << i << connector << j << connector << k << suffix;
				string fn = ss.str();
				cv::Mat image = imread(fn);
				vec.push_back(image);
			}
		}
	}
	return vec;
}
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



// a backup
/*Mat H2 = H.t();
double h2[3 + 1][3 + 1] = { 0 };
for (size_t i = 0; i < H2.rows; i++)
{
for (size_t j = 0; j < H2.cols; j++)
{
h2[i + 1][j + 1] = H2.at<double>(i, j);
}
}

Mat _A2 = (Mat_<double>(2, 2) << _c[1][1], _c[1][2], _c[2][1], _c[2][2]);
Mat _b2 = (Mat_<double>(2, 1) << -1.0 * _c[1][3], -1.0 * _c[2][3]);
Mat _x2 = _A2.t() * _b2;

double _t2 = 1;
Mat _V2 = (Mat_<double>(2, 5) << _a[1], _a[2], _a[3], _a[4], _a[5],
_b[1], _b[2], _b[3], _b[4], _b[5]);
Mat _X2 = (Mat_<double>(5, 1) << -_t2 * _t2 * prinPnt.x,
-prinPnt.y, _t2 * _t2, 1, _t2 * _t2 * prinPnt.x * prinPnt.x + prinPnt.y * prinPnt.y + _t2 * _t2 * 1000 * 1000);

Mat _bb2 = _V2 * _X2;

cout << "fy2: " << _x << endl << "fx2: " << _y << endl;


double V[2 + 1][6 + 1] = { 0 };

V[1][1] = h2[1][1] * h2[2][1];
V[1][2] = h2[1][1] * h2[2][2] + h2[1][2] * h2[2][1];
V[1][3] = h2[1][2] * h2[2][2];
V[1][4] = h2[1][3] * h2[2][1] + h2[1][1] * h2[2][3];
V[1][5] = h2[1][3] * h2[2][2] + h2[1][2] * h2[2][3];
V[1][6] = h2[1][3] * h2[2][3];

V[2][1] = h2[1][1] * h2[1][1] - h2[2][1] * h2[2][1];
V[2][2] = 2 * (h2[1][1] * h2[1][2] - h2[2][1] * h2[2][2]);
V[2][3] = h2[1][2] * h2[1][2] - h2[2][2] * h2[2][2];
V[2][4] = 2 * (h2[1][1] * h2[1][3] - h2[2][1] * h2[2][3]);
V[2][5] = 2 * (h2[1][2] * h2[1][3] - h2[2][2] * h2[2][3]);
V[2][6] = h2[1][3] * h2[1][3] - h2[2][3] * h2[2][3];

Mat V2 = (Mat_<double>(2, 6) <<
V[1][1], V[1][2], V[1][3], V[1][4], V[1][5], V[1][6],
V[2][1], V[2][2], V[2][3], V[2][4], V[2][5], V[2][6]
);

Mat S2, U2, VT2;
cv::SVD::compute(V2, S2, U2, VT2, SVD::FULL_UV);
double b2[6];
for (size_t i = 0; i < VT2.rows; i++)
{
b2[i] = VT2.at<double>(i, VT2.cols - 1);
}

double lamda = b2[5] - ( b2[3] * b2[3] + prinPnt.y * ( b2[1] * b2[3] - b2[0] * b2[4])) / b2[0];

double alpha = std::sqrt(lamda / b2[0]);


double b[2 + 1][3 + 1] = { 0 };
b[1][1] = V[1][1] - V[1][4] * prinPnt.x - V[1][6] * prinPnt.x * prinPnt.x;
b[1][2] = V[1][3] - V[1][5] * prinPnt.y + V[1][6] * prinPnt.y * prinPnt.y;
b[1][3] = V[1][6];
b[2][1] = V[2][1] - V[2][4] * prinPnt.x - V[2][6] * prinPnt.x * prinPnt.x;
b[2][2] = V[2][3] - V[2][5] * prinPnt.y + V[2][6] * prinPnt.y * prinPnt.y;
b[2][3] = V[2][6];

b[1][1] /= b[1][3];
b[1][2] /= b[1][3];
b[2][1] /= b[2][3];
b[2][2] /= b[2][3];

double rst1 = b[1][1] / h[1][1] + b[1][2] / h[2][2] + b[1][3];
double rst2 = b[2][1] / h[1][1] + b[2][2] / h[2][2] + b[2][3];

double _x1 = (b[1][2] * b[2][3] - b[1][3] * b[2][2]) / (b[2][2] * b[1][1] - b[2][1] * b[1][2]);
double _y1 = (b[1][1] * b[2][3] - b[1][3] * b[2][1]) / (b[1][2] * b[2][1] - b[1][1] * b[2][2]);

cout << "fy2: " << 1/ _x1 << endl << "fx2: " << 1 /_y1 << endl;


Mat x = (Mat_<double>(6, 1) <<
1.0 / (1000 * 1000 ), 0, 1.0 / (1000 * 1000),
-prinPnt.x / (1000 * 1000),
-prinPnt.y / (1000 * 1000),
-prinPnt.x * prinPnt.x / (1000 * 1000) + prinPnt.y * prinPnt.y / (1000 * 1000) + 1
);

Mat V32 = (Mat_<double>(2, 6) <<
V[1][1], V[1][2], V[1][3], V[1][4], V[1][5], V[1][6],
V[2][1], V[2][2], V[2][3], V[2][4], V[2][5], V[2][6]
);
Mat X1 = V2 * x;

cout << "lamda" << b[1][1] / b[2][1] << endl << b[1][2] / b[2][2] << endl << b[1][3] / b[2][3] << endl;
K = (Mat_<double>(3, 3) <<
std::sqrt(1 / _x1), 0, prinPnt.x,
0, std::sqrt(1 / _y1), prinPnt.y,
0, 0, 1
);


Mat A2 = (Mat_<double>(2, 6) <<
V[1][1], V[1][2], V[1][3], V[1][4], V[1][5], V[1][6],
V[2][1], V[2][2], V[2][3], V[2][4], V[2][5], V[2][6]
);

Mat S, U, VT;
cv::SVD::compute(A2, S, U, VT, SVD::FULL_UV);
double tmp = 1.0;


Mat A = (Mat_<double>(2, 2) <<
V[1][1] - V[1][4] * prinPnt.x - V[1][6] * prinPnt.x * prinPnt.x,
V[1][3] - V[1][5] * prinPnt.y + V[1][6] * prinPnt.y * prinPnt.y,
V[2][1] - V[2][4] * prinPnt.x - V[2][6] * prinPnt.x * prinPnt.x,
V[2][3] - V[2][5] * prinPnt.y + V[2][6] * prinPnt.y * prinPnt.y
);
Mat B = (Mat_<double>(2, 1) << V[1][6], V[2][6]		);

Mat X = (A.t() * A).inv() * A.t() * B;

cout << "X: " << X << endl << endl;*/
