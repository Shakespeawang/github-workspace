

#include <fstream>
#include <string>
#include <opencv.hpp>
#include <ceres.h>
#include <math.h>

using namespace cv;
using namespace ceres;
using namespace std;


const Size board_size = Size(9, 6);
const string IN_PATH = "D:\\毕业设计\\图漾第二次标定数据\\out2\\";
const string IN_IMAGE_PATH = "D:\\毕业设计\\图漾第二次标定数据\\";
const string OUT_PATH = "D:\\毕业设计\\图漾第二次标定数据\\out2\\out\\";
const int CAMERA_NUM = 11;
const int MAX_IMAGE_NUM = 11;
vector< vector<Mat>> _D_VIEW;


void optimize_all_observe(
	vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	const vector<vector<vector<double> > > S_Si, const vector<Point3d> object
	);

void output_image_point_cloud(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	const vector<vector<vector<double> > > S_Si,
	const string ss1,
	const bool is_have_si = true
	);
void output_all_point_and_loss(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos,
	const vector<Mat_<float>> pose_timestamp, const vector<Mat_<float>> distortion,
	const vector<vector<vector<double>>> S_Si, const vector<Point3d> object,
	const string ss1 = OUT_PATH + "反向投影点云____________________.txt",
	const string ss2 = OUT_PATH + "反向投影（由像素点到物体坐标点）.txt",
	const string ss3 = OUT_PATH + "__重投影（由物体坐标点到像素点）.txt",
	const bool is_have_si = true
	);


bool triangulation(vector<double> & s_0, vector<double> & s_1,
	const vector<Point3f> & observe0, const vector<Point3f> & observe1,
	const Mat & K0, const Mat & K1,
	const Mat & R
	);

void get_all_corner_point(vector < vector < vector < Point3f > > > & all_observe, Size board_size);
void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d> & corners);

bool readParams(string path, vector<Mat_<float>> & vec1, Size size1, string flag1, vector<Mat_<float>> & vec2, Size size2 = Size(0,0), string flag2 = "", int n = 11);

vector<Mat> readImage(string path, int _k, int _j, int _i = 0, string suffix = ".png", string connector = "-");
Mat readImage(int _i, int _j, int _k = 0, string path = IN_IMAGE_PATH, string suffix = ".png", string connector = "-");

bool writeImage(Mat image, int _i, int _j, int _k = 0, string path = OUT_PATH, string suffix = ".png", string connector = "-");

bool out_all_point(const vector < vector < vector < Point3f > > > & all_observe, const string ss1 = OUT_PATH + "棋盘格角点.txt");

template<typename T>
T min(T a, T b)
{
	return a > b ? b : a;
}

int main(int argc, char * argv[])
{

	fstream fcout(OUT_PATH + "console__report.txt", ios::out);


	vector< Mat_<float> >camera_pos, pose_timestamp, distortion, intrinsic;
	vector< Mat_<float> >optimized_camera_pos, optimized_pose_timestamp, optimized_distortion, optimized_intrinsic;
	camera_pos.clear(), pose_timestamp.clear(), distortion, intrinsic.clear();
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();

	//读入参数
	readParams(IN_PATH + "intrinsic.xml", intrinsic, Size(3, 3), "K_", distortion, Size(1, 5), "DISTORTION_");
	readParams(IN_PATH + "ex0.xml", pose_timestamp, Size(4, 4), "pose_timestamp_", pose_timestamp);
	readParams(IN_PATH + "exito0.xml", camera_pos, Size(4, 4), "camera_pose_", camera_pos);

	//读入棋盘格所有的角点
	vector < vector < vector < Point3f > > > all_observe; //z用作是否参与运算的flag
	all_observe.clear();
	Size board_size = Size(9, 6);
	get_all_corner_point(all_observe, board_size);
	out_all_point(all_observe);

	vector<Point3d> object;
	object.clear();
	double squareSize = 50;
	calcChessboardCorners(board_size, squareSize, object);

	Mat K0 = intrinsic[0];
	vector< vector< vector <double> > > S_Si, S_S0;
	S_Si.clear(), S_S0.clear();
	//求出si
	for (int camera_i = 1; camera_i < all_observe.size(); camera_i++) //(0,1),(0,2),...,(0,10)
	{
		vector< vector<double> > s_0, s_1;
		s_0.clear(), s_1.clear();

		for (int image_j = 0; image_j < all_observe[camera_i].size(); image_j++) {

			Mat K1 = intrinsic[camera_i];
			vector<double> tmp_s0, tmp_s1;
			tmp_s0.clear();
			tmp_s1.clear();
			//零值三角测量
			//get_s0_s1(tmp_s0, tmp_s1, all_observe[0][image_j], all_observe[camera_i][image_j], K0, K1, camera_pos[camera_i]);
			//最小二乘三角测量
			triangulation(tmp_s0, tmp_s1, all_observe[0][image_j], all_observe[camera_i][image_j], K0, K1, camera_pos[camera_i]);
			s_0.push_back(tmp_s0);
			s_1.push_back(tmp_s1);

		}
		S_S0.push_back(s_0);
		S_Si.push_back(s_1);

	}

	S_Si.insert(S_Si.begin() + 0, S_S0[0]);//求出Si

	output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, S_Si, object,
		OUT_PATH + "优化前__反向投影点云____________________.txt",
		OUT_PATH + "优化前__反向投影（由像素点到物体坐标点）.txt",
		OUT_PATH + "优化前____重投影（由物体坐标点到像素点）.txt"); // before optimizating

	output_image_point_cloud(all_observe, intrinsic, camera_pos, pose_timestamp, S_Si, OUT_PATH + "图像点云"); // before optimizating




	//optimize_all_observe(all_observe, intrinsic, camera_pos, pose_timestamp, S_Si, object); // before optimizating
	//
	//output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, S_Si, object,
	//	OUT_PATH + "去掉误差较大的点后__反向投影点云____________________.txt",
	//	OUT_PATH + "去掉误差较大的点后__反向投影（由像素点到物体坐标点）.txt",
	//	""); // before optimizating

	fcout.close();
	system("pause");
	return 0;
}




void output_image_point_cloud(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	const vector<vector<vector<double> > > S_Si,
	const string ss1,
	const bool is_have_si
	)
{
	for (int Camera_i = 0; Camera_i < all_observe.size(); Camera_i++) {

		for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++) {

			stringstream ss;
			ss << ss1 << "__camera_" << Camera_i << "__image_" << image_j << ".txt";

			fstream fs1(ss.str(), ios::out);

			Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			for (int i = 0; i < 4; i++) {
				float * ptr = tmp_intrinsic_i.ptr<float>(i);
				for (int j = 0; j < 4; j++) {
					if (i < 3 && j < 3)
						ptr[j] = intrinsic[Camera_i].at<float>(i, j);
					else if (3 == i && 3 == j)
						ptr[j] = 1.0;
					else
						ptr[j] = 0.0;
				}
			}

			Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo = tmp_intrinsic_i * camera_pos[Camera_i];
			Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo_inv = homo.inv();

			for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++) {

				//反向投影
				if (is_have_si) {
					if (S_Si[Camera_i][image_j].size() <= 0) {
						continue;
					}
				}
				bool flag = all_observe[Camera_i][image_j][k].z;//是否参与计算
				if (flag) {
					Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					tmp_image_point.at<float>(0, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].x;
					tmp_image_point.at<float>(1, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].y;
					tmp_image_point.at<float>(2, 0) = S_Si[Camera_i][image_j][k];
					tmp_image_point.at<float>(3, 0) = 1.0;

					Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					possible_object_point = homo_inv * tmp_image_point;

					Point3d object_point;
					object_point.x = possible_object_point.at<float>(0, 0);
					object_point.y = possible_object_point.at<float>(1, 0);
					object_point.z = possible_object_point.at<float>(2, 0);


					fs1 << object_point.x << " " << object_point.y << " " << object_point.z << endl;
				}
			}
			fs1.close();
		}
	}

}


bool sort_min(Point2f p1, Point2f p2)
{
	return p1.x < p2.x;
}


void optimize_all_observe(
	vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	const vector<vector<vector<double> > > S_Si, const vector<Point3d> object
	)
{
	for (int Camera_i = 0; Camera_i < all_observe.size(); Camera_i++) {

		for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++) {

			Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			for (int i = 0; i < 4; i++) {
				float * ptr = tmp_intrinsic_i.ptr<float>(i);
				for (int j = 0; j < 4; j++) {
					if (i < 3 && j < 3)
						ptr[j] = intrinsic[Camera_i].at<float>(i, j);
					else if (3 == i && 3 == j)
						ptr[j] = 1.0;
					else
						ptr[j] = 0.0;
				}
			}

			Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo = tmp_intrinsic_i * camera_pos[Camera_i] * pose_timestamp[image_j];
			Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo_inv = homo.inv();

			double sum_loss3 = 0.0, half_sum_loss3 = 0.0;

			if (S_Si[Camera_i][image_j].size() <= 0) {
				continue;
			}
			vector<Point2d > error_sum;
			error_sum.clear();
			for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++) {


				//反向投影
				Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				tmp_image_point.at<float>(0, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].x;
				tmp_image_point.at<float>(1, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].y;
				tmp_image_point.at<float>(2, 0) = S_Si[Camera_i][image_j][k];
				tmp_image_point.at<float>(3, 0) = 1.0;

				Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				possible_object_point = homo_inv * tmp_image_point;

				Point3d object_point;
				object_point.x = possible_object_point.at<float>(0, 0);
				object_point.y = possible_object_point.at<float>(1, 0);
				object_point.z = possible_object_point.at<float>(2, 0);

				sum_loss3 = pow(object_point.x - object[k].x, 2)
					+ pow(object_point.y - object[k].y, 2)
					+ pow(object_point.z - object[k].z, 2);

				Point2d pnt(sum_loss3, k);

				error_sum.push_back(pnt);
			}
			sort(error_sum.begin(), error_sum.end(), sort_min);

			for (int k = error_sum.size() - 1; k >= 0 && k > error_sum.size() - 1 - board_size.height; --k) {
				int idx = (int)error_sum[k].y;
				all_observe[Camera_i][image_j][idx].z = false;
			}

		}
	}
}

bool out_all_point(const vector < vector < vector < Point3f > > > & all_observe, const string ss1)
{
	fstream fs1(ss1, ios::out);
	if (!fs1.is_open())
		return false;

	int point_cnt = 0;
	int flag_cnt = 0;
	for (int _i = 0; _i < all_observe.size(); _i++)
	{
		fs1 << "\n\n\n\n\n第" << _i << "个相机：\n";

		for (int _j = 0; _j < all_observe[_i].size(); _j++) {

			fs1 << "\n\t第" << _j << "张图片：\n";

			for (int k = 0; k < all_observe[_i][_j].size(); k++) {
				point_cnt++;
				bool flag = all_observe[_i][_j][k].z;
				if (!flag)
					flag_cnt++;//如果flag为false,则记录不参与计算的点的个数
				fs1 << "\t" << k << ",\tx: " << all_observe[_i][_j][k].x
					<< ",\ty: " << all_observe[_i][_j][k].y
					<< ",\tz(flag): " << all_observe[_i][_j][k].z << endl;
			}

		}
	}
	fs1 << "\n\n总共: " << point_cnt << "点\n";
	fs1 << "\n\n不参与计算的有: " << flag_cnt << "点\n";
	fs1 << "\n\n参与计算的有: " << (point_cnt - flag_cnt) << "点\n";
	return true;
}


void get_all_corner_point(vector < vector < vector < Point3f > > > & all_observe, Size board_size)
{
#ifdef _DEBUG
	_D_VIEW.clear();
#endif

	all_observe.clear();
	for (int _i = 0; _i < CAMERA_NUM; _i++)
	{
		vector<Mat> vec_image_i = readImage(IN_IMAGE_PATH, MAX_IMAGE_NUM, _i);

		vector<vector<Point3f>> observe_s_j;
		observe_s_j.clear();

		for (int _j = 0; _j < MAX_IMAGE_NUM; _j++) {

			vector<Point2f> observe;
			vector<Point3f> observe3f;
			observe.clear(), observe3f.clear();
			bool found = findChessboardCorners(vec_image_i[_j], board_size, observe, CALIB_CB_ADAPTIVE_THRESH);
			if (!found) {
				observe.clear();
			}
			for (int i = 0; i < observe.size(); i++) {
				Point3f pnt(observe[i].x, observe[i].y, true); //z用来作为判断,flag变量
				observe3f.push_back(pnt);
			}
			observe_s_j.push_back(observe3f);

#ifdef _DEBUG
			Mat img0;
			//vec_image_i[_j].copyTo(img0);
			//cv::cvtColor(vec_image0[_j], img0, CV_GRAY2RGB);
			cv::drawChessboardCorners(vec_image_i[_j], board_size, observe, found);
			img0 = vec_image_i[_j];
#endif // _DEBUG

		}
#ifdef _DEBUG
		_D_VIEW.push_back(vec_image_i);
#endif
		all_observe.push_back(observe_s_j);
	}
}



void output_all_point_and_loss(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos,
	const vector<Mat_<float>> pose_timestamp, const vector<Mat_<float>> distortion,
	const vector<vector<vector<double> > > S_Si, const vector<Point3d> object,
	const string ss1,
	const string ss2,
	const string ss3,
	const bool is_have_si
	)
{
	fstream fs1(ss1, ios::out);
	fstream fs2(ss2, ios::out);
	fstream fs3(ss3, ios::out);

	double sum_reprojection_loss1 = 0.0, half_sum_reprojection_loss1 = 0.0;
	double sum_loss1 = 0.0, half_sum_loss1 = 0.0;
	for (int Camera_i = 0; Camera_i < all_observe.size(); Camera_i++) {

		fs2 << "\n\n\n\n\n第" << Camera_i << "个相机：\n";
		fs3 << "\n\n\n\n\n第" << Camera_i << "个相机：\n";

		double sum_reprojection_loss2 = 0.0, half_sum_reprojection_loss2 = 0.0;
		double sum_loss2 = 0.0, half_sum_loss2 = 0.0;

		for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++) {

			fs2 << "\n\t第" << image_j << "张图片：\n";
			fs3 << "\n\t第" << image_j << "张图片：\n";

#ifdef _DEBUG
			Mat _D_02 = pose_timestamp[Camera_i];
#endif // _DEBUG

			Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			for (int i = 0; i < 4; i++) {
				float * ptr = tmp_intrinsic_i.ptr<float>(i);
				for (int j = 0; j < 4; j++) {
					if (i < 3 && j < 3)
						ptr[j] = intrinsic[Camera_i].at<float>(i, j);
					else if (3 == i && 3 == j)
						ptr[j] = 1.0;
					else
						ptr[j] = 0.0;
				}
			}
			double fx = intrinsic[Camera_i].at<float>(0, 0);
			double cx = intrinsic[Camera_i].at<float>(0, 2);
			double fy = intrinsic[Camera_i].at<float>(1, 1);
			double cy = intrinsic[Camera_i].at<float>(1, 2);

			Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo = camera_pos[Camera_i] * pose_timestamp[image_j];
			Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo_inv = (tmp_intrinsic_i * homo).inv();

			vector<Point2f> vec_reprojection;
			vec_reprojection.clear();

			double sum_reprojection_loss3 = 0.0, half_sum_reprojection_loss3 = 0.0;
			double sum_loss3 = 0.0, half_sum_loss3 = 0.0;

			for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++) {
#ifdef _DEBUG
				Mat _D_03 = _D_VIEW[Camera_i][image_j];
#endif // _DEBUG

				//重投影
				Mat tmp_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				tmp_object_point.at<float>(0, 0) = object[k].x;
				tmp_object_point.at<float>(1, 0) = object[k].y;
				tmp_object_point.at<float>(2, 0) = object[k].z;
				tmp_object_point.at<float>(3, 0) = 1.0;

				Mat possible_pixel_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				possible_pixel_point = homo * tmp_object_point;

				Point3d reprojection_point;
				double x = reprojection_point.x = possible_pixel_point.at<float>(0, 0) / possible_pixel_point.at<float>(2, 0);
				double y = reprojection_point.y = possible_pixel_point.at<float>(1, 0) / possible_pixel_point.at<float>(2, 0);
				reprojection_point.z = possible_pixel_point.at<float>(2, 0);

				//Mat KKKKKKKKK = distortion[Camera_i];
				double k1 = distortion[Camera_i].at<float>(0, 0);
				double k2 = distortion[Camera_i].at<float>(0, 1);
				double k3 = distortion[Camera_i].at<float>(0, 2);
				double p1 = distortion[Camera_i].at<float>(0, 3);
				double p2 = distortion[Camera_i].at<float>(0, 4);

				double r2 = x * x + y * y;

				double d_k = (1.0) + k1 * r2 + k2 * pow(r2, 2) + k3 * pow(r2, 3);

				double x_distorted = x * d_k + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
				double y_distorted = y * d_k + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

				reprojection_point.x = fx * x_distorted + cx;
				reprojection_point.y = fy * y_distorted + cy;

				Point2f reprojection_point_2f(reprojection_point.x, reprojection_point.y);
				vec_reprojection.push_back(reprojection_point_2f);

				sum_reprojection_loss3 = sum_reprojection_loss3 + pow(reprojection_point.x - all_observe[Camera_i][image_j][k].x, 2)
					+ pow(reprojection_point.y - all_observe[Camera_i][image_j][k].y, 2);

				half_sum_reprojection_loss3 = half_sum_reprojection_loss3 + 0.5 * pow(reprojection_point.x - all_observe[Camera_i][image_j][k].x, 2)
					+ 0.5 * pow(reprojection_point.y - all_observe[Camera_i][image_j][k].y, 2);

				fs3 << "\t" << k << ",\tx: " << reprojection_point.x << ",\ty: " << reprojection_point.y << ",\tz(hold,may be si): " << reprojection_point.z << endl;


				//反向投影
				if (is_have_si) {
					if (S_Si[Camera_i][image_j].size() <= 0) {
						continue;
					}
				}
				bool flag = all_observe[Camera_i][image_j][k].z;//是否参与计算
				if (flag) {
					Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					tmp_image_point.at<float>(0, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].x;
					tmp_image_point.at<float>(1, 0) = S_Si[Camera_i][image_j][k] * all_observe[Camera_i][image_j][k].y;
					tmp_image_point.at<float>(2, 0) = S_Si[Camera_i][image_j][k];
					tmp_image_point.at<float>(3, 0) = 1.0;

					Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					possible_object_point = homo_inv * tmp_image_point;

					Point3d object_point;
					object_point.x = possible_object_point.at<float>(0, 0);
					object_point.y = possible_object_point.at<float>(1, 0);
					object_point.z = possible_object_point.at<float>(2, 0);

					sum_loss3 = sum_loss3 + pow(object_point.x - object[k].x, 2)
						+ pow(object_point.y - object[k].y, 2)
						+ pow(object_point.z - object[k].z, 2);

					half_sum_loss3 = half_sum_loss3 + 0.5 * pow(object_point.x - object[k].x, 2)
						+ 0.5 * pow(object_point.y - object[k].y, 2)
						+ 0.5 * pow(object_point.z - object[k].z, 2);


					fs1 << object_point.x << " " << object_point.y << " " << object_point.z << endl;

					fs2 << "\t" << k << ",\tx: " << object_point.x << ",\ty: " << object_point.y << ",\tz: " << object_point.z << endl;
				}


			}
			sum_loss2 = sum_loss2 + sum_loss3;
			half_sum_loss2 = half_sum_loss2 + half_sum_loss3;

			sum_reprojection_loss2 = sum_reprojection_loss2 + sum_reprojection_loss3;
			half_sum_reprojection_loss2 = half_sum_reprojection_loss2 + half_sum_reprojection_loss3;

			fs2 << "\n\tIMAGE SUM: sum_loss: " << sum_loss3 << ", 0.5 * sum_loss: " << half_sum_loss3 << endl;
			fs3 << "\n\tIMAGE SUM: sum_loss: " << sum_reprojection_loss3 << ", 0.5 * sum_loss: " << half_sum_reprojection_loss3 << endl;


			Mat image = readImage(0, Camera_i, image_j);
			if (image.empty())
				continue;
			cv::drawChessboardCorners(image, board_size, vec_reprojection, true);
			string ssstr = "";
			if (OUT_PATH.size() > ss2.size() - 1 || ss2.size() < OUT_PATH.size())
				ssstr = "";
			else
				ss2.substr(OUT_PATH.size(), ss2.size() - OUT_PATH.size());
			writeImage(image, 0, Camera_i, image_j, OUT_PATH + "image\\", ssstr + "_reprojection.png");
		}
		sum_loss1 = sum_loss1 + sum_loss2;
		half_sum_loss1 = half_sum_loss1 + half_sum_loss2;

		sum_reprojection_loss1 = sum_reprojection_loss1 + sum_reprojection_loss2;
		half_sum_reprojection_loss1 = half_sum_reprojection_loss1 + half_sum_reprojection_loss2;

		fs2 << "\nCAMERA SUM: sum_loss: " << sum_loss2 << ", 0.5 * sum_loss: " << half_sum_loss2 << endl;
		fs3 << "\nCAMERA SUM: sum_loss: " << sum_reprojection_loss2 << ", 0.5 * sum_loss: " << half_sum_reprojection_loss2 << endl;

	}

	fs2 << "\nALL SUM: sum_loss: " << sum_loss1 << ", 0.5 * sum_loss: " << half_sum_loss1 << endl;
	fs3 << "\nALL SUM: sum_loss: " << sum_reprojection_loss1 << ", 0.5 * sum_loss: " << half_sum_reprojection_loss1 << endl;


	fs1.close();
	fs2.close();
	fs3.close();
}


bool readParams(string path, vector<Mat_<float>> & vec1, Size size1, string flag1, vector<Mat_<float>> & vec2, Size size2, string flag2, int n)
{
	FileStorage fs(path, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < n; i++)
	{
		if (flag2.empty()){
			stringstream ss1;
			ss1 << flag1 << i;
			Mat mat1(size1, CV_32FC1);
			fs[ss1.str()] >> mat1;
			if (!mat1.empty())
				vec1.push_back(mat1);
		}
		else{
			stringstream ss1, ss2;
			ss1 << flag1 << i;
			ss2 << flag2 << i;
			Mat mat1(size1, CV_32FC1);
			Mat mat2(size2, CV_32FC1);
			fs[ss1.str()] >> mat1;
			fs[ss2.str()] >> mat2;
			if (!mat1.empty())
				vec1.push_back(mat1);
			if (!mat2.empty())
				vec2.push_back(mat2);
		}
	}
	fs.release();
	return true;
}

void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d> & corners)
{
	corners.resize(0);
	for (int i = 0; i < boardSize.height; i++) {     //height和width位置不能颠倒
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(Point3d(j*squareSize, i*squareSize, 0));
		}
	}
}

vector<Mat> readImage(string path, int _k, int _j, int _i, string suffix, string connector)
{
	vector<Mat> vec;
	vec.clear();
	for (int i = _i; i <= _i; i++) {
		for (int j = _j; j <= _j; j++) {
			for (int k = 0; k < _k; k++) {
				stringstream filename;
				filename << path << i << connector << j << connector << k << suffix;
				Mat image = imread(filename.str());
				vec.push_back(image);
			}
		}
	}
	return vec;
}

Mat readImage(int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	Mat image = imread(filename.str());
	return image;
}

bool writeImage(Mat image, int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	return imwrite(filename.str(), image);
}

Point2d pixel2cam(Point2d pix_pnt, Mat K)
{
	Mat pnt = (Mat_<float>(3, 1) << pix_pnt.x, pix_pnt.y, 1.0);
	Mat cam_pnt = K.inv() * pnt;
	Point2d pt;
	pt.x = cam_pnt.at<float>(0, 0);
	pt.y = cam_pnt.at<float>(1, 0);
	return pt;
}

Point3d cam1_to_cam2(Point3d pnt, Mat_<double> R)
{
	Mat m_pnt = (Mat_<double>(4, 1) << pnt.x, pnt.y, pnt.z, 1.0);
	Mat cam_pnt = R * m_pnt;
	Point3d pt;
	pt.x = cam_pnt.at<double>(0, 0);
	pt.y = cam_pnt.at<double>(1, 0);
	pt.z = cam_pnt.at<double>(2, 0);
	return pt;
}

bool triangulation(vector<double> & s_0, vector<double> & s_1,
	const vector<Point3f> & observe0, const vector<Point3f> & observe1,
	const Mat & K0, const Mat & K1,
	const Mat & R
	)
{
	s_0.clear(), s_1.clear();
	if (observe0.size() != observe1.size() || observe0.size() <= 0 || observe1.size() <= 0)
		return false;
	vector<Point2d> pts_1, pts_2;
	for (int i = 0; i < observe0.size(); i++){
		Point2d pn0(observe0[i].x, observe0[i].y);
		Point2d pn1(observe1[i].x, observe1[i].y);
		pts_1.push_back(pixel2cam(pn0, K0));
		pts_2.push_back(pixel2cam(pn1, K1));
	}

	Mat T1 = (Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat T2 = (Mat_<double>(3, 4) <<
		R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), R.at<float>(0, 3),
		R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), R.at<float>(1, 3),
		R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), R.at<float>(2, 3)
		);

	Mat pts_4d;
	cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
	vector<Point3d> points0, points1;
	for (int i = 0; i < pts_4d.cols; i++){
		Mat x = pts_4d.col(i);
		x /= x.at<double>(3, 0);
		Point3d p(
			x.at<double>(0, 0),
			x.at<double>(1, 0),
			x.at<double>(2, 0)
			);
		Point3d p1 = cam1_to_cam2(p, T2);
		points0.push_back(p);
		points1.push_back(p1);
		s_0.push_back(p.z);
		s_1.push_back(p1.z);
	}

	return true;
}

