

#include <fstream>
#include <string>
#include <opencv.hpp>
#include <ceres.h>
#include <math.h>
#include <cmath>
#include "optimizationFunction.h"

using namespace cv;
using namespace ceres;
using namespace std;


const int CAMERA_NUM = 11;
const int MAX_IMAGE_NUM = 11;
const Size board_size = Size(9, 6);
fstream fcout;
vector< vector<Mat>> _D_VIEW;


void output_all_point_and_loss(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos,
	const vector<Mat_<float>> pose_timestamp, const vector<Mat_<float>> distortion,
	const vector<vector<vector<Point3d>>> recon_pts, const vector<Point3d> object,
	const string ss1 = OUT_PATH + "反向投影点云____________________.txt",
	const string ss2 = OUT_PATH + "反向投影（由像素点到物体坐标点）.txt",
	const string ss3 = OUT_PATH + "__重投影（由物体坐标点到像素点）.txt",
	const bool is_have_si = true
	);


bool triangulation(vector<Point3d> & pts,const vector<Point3f> & observe0, const vector<Point3f> & observe1, const Mat & K0, const Mat & K1,const Mat & R);
void get_all_corner_point(vector < vector < vector < Point3f > > > & all_observe, Size board_size);
void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d> & corners);

template<typename T>
bool matrix_multiply(const T src1[], const T src2[], T des[], int src1_row, int src1_col, int src2_row, int src2_col)
{
	if (src1_col != src2_row)
		return false;
	for (int i = 0; i < src1_row; i++) {
		for (int j = 0; j < src2_col; j++) {
			T sum = (T)0.0;
			for (int k = 0; k < src1_col; k++) {
#ifdef _DEBUG
				int x = i * src1_col + k;
				int y = k * src2_col + j;
				int z = i * src2_col + j;
#endif // _DEBUG

				sum += src1[i * src1_col + k] * src2[k * src2_col + j];
			}
			des[i * src2_col + j] = sum;
		}
	}
	return true;
}

template<typename T>
T min(T a, T b)
{
	return a > b ? b : a;
}
template<typename T>
bool matrix_resize(const T src[], T des[], int src_row, int src_col, int des_row, int des_col)
{
	if (src_row > des_row || src_col > des_col)
		return false;
	int x = src_row == des_row ? src_col : 0;
	for (int i = 0; i < des_row; i++) {
		for (int j = 0; j < des_col; j++) {
			if (j < src_col && i < src_row) {
				des[i * des_col + j] = (T)src[i * src_col + j];
			}
			else if (i == j - x)
				des[i * des_col + j] = (T)1.0;
			else
				des[i * des_col + j] = (T)0.0;
		}
	}
	return true;
}

template<typename T>
bool matrix_inv(const T src[], T des[], int src_row, int src_col)
{
	T * re_src = new T[src_row * src_col * 2];
	/*T a[16] = {
	(T)2,(T)2,(T)1,(T)4,
	(T)4,(T)4,(T)2,(T)1,
	(T)2,(T)4,(T)1,(T)1,
	(T)1,(T)1,(T)4,(T)2,
	}; */
	//T re_src[2 * 16];
	int col = src_col * 2;
	matrix_resize(src, re_src, src_row, src_col, src_row, col);
	for (int i = 0; i < src_row; i++) {
		T temp = re_src[i * col + i];
		if (!(re_src[i * col + i] >(T)0 || re_src[i * col + i] < (T)0)) {
			int j = i + 1;
			for (j = i + 1; j < src_row; j++) {
				if (re_src[j * col + i] >(T)0 || re_src[j * col + i] < (T)0) {
					for (int k = 0; k < col; k++) {
						re_src[i * col + k] += re_src[j * col + k];
					}
					break;
				}
			}
			if (j >= src_row)
				return false;
		}
		temp = re_src[i * col + i];
		for (int j = 0; j < col; j++) {
			re_src[i * col + j] /= temp;
		}
		for (int k = i + 1; k < src_row; k++) {
			T temp = re_src[k * col + i];
			for (int m = col - 1; m >= 0; --m) {
				re_src[k * col + m] -= temp * re_src[i * col + m];
			}
		}
	}
	for (int i = 1; i < src_row; i++) {
		for (int k = 0; k < i; k++) {
			T temp = re_src[k * col + i];
			for (int m = 0; m < col; ++m) {
				re_src[k * col + m] -= temp * re_src[i * col + m];
			}
		}
	}
	for (int i = 0; i < src_row; i++) {
		for (int j = col - src_col; j < col; j++) {
			des[i * src_col + j - src_col] = re_src[i * col + j];
		}
	}
	delete re_src;
	return true;
}


template<typename T>
void CalcRodrigues(const T a[3], T b[9])
{
	if (a[0] == (T)0.0&&a[1] == (T)0.0&&a[2] == (T)0.0)
	{
		b[0] = (T)1.0;
		b[1] = (T)0.0;
		b[2] = (T)0.0;
		b[3] = (T)0.0;
		b[4] = (T)1.0;
		b[5] = (T)0.0;
		b[6] = (T)0.0;
		b[7] = (T)0.0;
		b[8] = (T)1.0;
	}
	else
	{
		T angle = ceres::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		T u[3] = { a[0] / angle, a[1] / angle, a[2] / angle };
		b[0] = ceres::cos(angle) + u[0] * u[0] * (1.0 - ceres::cos(angle));
		b[1] = u[0] * u[1] * (1.0 - ceres::cos(angle)) - u[2] * ceres::sin(angle);
		b[2] = u[1] * ceres::sin(angle) + u[0] * u[2] * (1.0 - ceres::cos(angle));

		b[3] = u[2] * ceres::sin(angle) + u[0] * u[1] * (1.0 - ceres::cos(angle));
		b[4] = ceres::cos(angle) + u[1] * u[1] * (1.0 - ceres::cos(angle));
		b[5] = -u[0] * ceres::sin(angle) + u[1] * u[2] * (1.0 - ceres::cos(angle));

		b[6] = -u[1] * ceres::sin(angle) + u[0] * u[2] * (1.0 - ceres::cos(angle));
		b[7] = u[0] * ceres::sin(angle) + u[1] * u[2] * (1.0 - ceres::cos(angle));
		b[8] = ceres::cos(angle) + u[2] * u[2] * (1.0 - ceres::cos(angle));
	}

}

template<typename T>
void RT2TransformTemplate(const T R[3], const T t[3], T Transform[])
{
	T mR[9];
	CalcRodrigues(R, mR);
	matrix_resize(mR, Transform, 3, 3, 4, 4);
	Transform[0 * 4 + 3] = t[0];
	Transform[1 * 4 + 3] = t[1];
	Transform[2 * 4 + 3] = t[2];
}

struct ReconstrutionError {
	ReconstrutionError(Point3d observe, Point3d objection, Point3d si)
	: observe_(observe), objection_(objection), si_(si) {}

	template <typename T>
	bool operator()(const T * const K, const T * const R2, const T * const t2, const T * const R1, const T * const t1, const T * const distortion, T * residual) const
	{
		T camera_pos[4 * 4], pose_timestamp[4 * 4];
		RT2TransformTemplate(R2, t2, camera_pos);
		RT2TransformTemplate(R1, t1, pose_timestamp);


		T homo1[4 * 4 + 0], homo2[4 * 4 + 0], homo_inv[4 * 4 + 0];
		T intrinsic[4 * 4] = { 
			K[0],	T(0),	K[2],	T(0),
			T(0),	K[1],	K[3],	T(0),
			T(0),	T(0),	T(1),	T(0),
			T(0),	T(0),	T(0),	T(1)
		};


		matrix_multiply(intrinsic, camera_pos, homo1, 4, 4, 4, 4);
		matrix_multiply(homo1, pose_timestamp, homo2, 4, 4, 4, 4);

		matrix_inv(homo2, homo_inv, 4, 4);

		T tmp_image_point[4 * 1];
		tmp_image_point[0 + 0 * 4] = T(si_.z * observe_.x);
		tmp_image_point[1 + 0 * 4] = T(si_.z * observe_.y);
		tmp_image_point[2 + 0 * 4] = T(si_.z);
		tmp_image_point[3 + 0 * 4] = T(1.0);

		T possible_objection_point[4 * 1 + 0];
		matrix_multiply(homo_inv, tmp_image_point, possible_objection_point, 4, 4, 4, 1);

		residual[0] = (T)possible_objection_point[0] - T(objection_.x);
		residual[1] = (T)possible_objection_point[1] - T(objection_.y);
		residual[2] = (T)possible_objection_point[2] - T(objection_.z);

		return true;
	}

	static ceres::CostFunction * Create(Point3d observe, Point3d objection, Point3d si) {

		return (new ceres::AutoDiffCostFunction<ReconstrutionError, 3, 4, 3, 3, 3, 3, 5>
			(new ReconstrutionError(observe, objection, si)));

	}
	Point3d si_;
	Point3d observe_;
	Point3d objection_;
};

struct ReprojectionError {
	ReprojectionError(Point3d observe, Point3d objection)
	: observe_(observe), objection_(objection) {}

	template <typename T>
	bool operator()(const T * const K, const T * const R2, const T * const t2, const T * const R1, const T * const t1, const T * const distortion, T * residual) const
	{
		T camera_pos[4 * 4], pose_timestamp[4 * 4];
		RT2TransformTemplate(R2, t2, camera_pos);
		RT2TransformTemplate(R1, t1, pose_timestamp);

		T homo[4 * 4 + 0];
		matrix_multiply(camera_pos, pose_timestamp, homo, 4, 4, 4, 4);

		T tmp_image_point[4 * 1];
		tmp_image_point[0] = T(objection_.x);
		tmp_image_point[1] = T(objection_.y);
		tmp_image_point[2] = T(objection_.z);
		tmp_image_point[3] = T(1.0);

		T reprojection_point[4 * 1 + 0];
		matrix_multiply(homo, tmp_image_point, reprojection_point, 4, 4, 4, 1);

		T x, y;
		x = reprojection_point[0] / reprojection_point[2];
		y = reprojection_point[1] / reprojection_point[2];//归一化坐标

		T r2 = x * x + y * y;
		T k_p = T(1.0) + distortion[0] * r2 + distortion[1] * r2 * r2 + distortion[2] * r2 * r2 * r2;

		T x_distorted, y_distorted;
		x_distorted = x * k_p + T(2.0) * distortion[3] * x * y + distortion[4] * (r2 + T(2.0) * x * x);
		y_distorted = y * k_p + T(2.0) * distortion[4] * x * y + distortion[3] * (r2 + T(2.0) * y * y);

		T fx = K[0], fy = K[1], cx = K[2], cy = K[3];

		T predicited_x = fx * x_distorted + cx;
		T predicited_y = fy * y_distorted + cy;

		residual[0] = (T)predicited_x - T(observe_.x);
		residual[1] = (T)predicited_y - T(observe_.y);

		return true;
	}

	static ceres::CostFunction * Create(Point3d observe, Point3d objection) {

		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3, 3, 3, 5>
			(new ReprojectionError(observe, objection)));

	}
	Point3d observe_;
	Point3d objection_;
};


void reprojection_optimize(const vector < vector < vector < Point3f > > > & all_observe, const vector<Point3d> & object)
{
	vector< Mat_<float> >camera_pos, pose_timestamp, distortion, intrinsic;
	vector< Mat_<float> >optimized_camera_pos, optimized_pose_timestamp, optimized_distortion, optimized_intrinsic;
	camera_pos.clear(), pose_timestamp.clear(), distortion, intrinsic.clear();
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();

	//读入参数
	readParams(IN_PATH + "cameras_intrinsic_parameters1.xml", intrinsic, Size(3, 3), "K_", distortion, Size(1, 5), "DISTORTION_");
	readParams(IN_PATH + "multi-extrinsic-results1.xml", camera_pos, Size(4, 4), "camera_pose_", camera_pos);
	readParams(IN_PATH + "multi-extrinsic-results1.xml", pose_timestamp, Size(4, 4), "pose_timestamp_", pose_timestamp);


	vector< vector< vector <Point3d> > > recon_pts;
	vector< vector< vector < vector <Point3d> > > > v4_pts;
	recon_pts.clear(), v4_pts.clear();
	//求出si
	for (int i = 0; i < all_observe.size(); i++) //(0,1),(0,2),...,(0,10);(1,1),(1,2)...,(1,10);...;(9,10);
	{
		vector< vector<Point3d> > v3pts;
		v3pts.clear();

		Mat K0 = intrinsic[i];

		for (int j = 0; j < all_observe[i].size(); j++) {

			vector< vector<Point3d> > v2pts;
			v2pts.clear();
			for (int k = 0; k < CAMERA_NUM; k++){
				if (k == i)continue;

				Mat K1 = intrinsic[k];
				vector<Point3d> tmp_pts;
				tmp_pts.clear();

				Mat R = camera_pos[k] * camera_pos[i].inv();
				//最小二乘三角测量
				triangulation(tmp_pts, all_observe[i][j], all_observe[k][j], K0, K1, R);
				v2pts.push_back(tmp_pts);
			}
			v3pts.push_back(optimize_array(v2pts));
		}
		recon_pts.push_back(v3pts);
	}

	output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, recon_pts, object,
		OUT_PATH + "reprojection_optimize优化前__反向投影点云____________________.txt",
		OUT_PATH + "reprojection_optimize优化前__反向投影（由像素点到物体坐标点）.txt",
		OUT_PATH + "reprojection_optimize优化前____重投影（由物体坐标点到像素点）.txt"); // before optimizating

	output_image_point_cloud(all_observe, intrinsic, camera_pos, pose_timestamp, recon_pts, OUT_PATH + "reprojection_optimize图像点云"); // before optimizating

	//optimize_all_observe(all_observe, intrinsic, camera_pos, pose_timestamp, recon_pts, object,board_size); // before optimizating
	//output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, recon_pts, object,
	//	OUT_PATH + "去掉误差较大的点后__反向投影点云____________________.txt",
	//	OUT_PATH + "去掉误差较大的点后__反向投影（由像素点到物体坐标点）.txt",
	//	""); // before optimizating

	double K[CAMERA_NUM][4];
	double R2[CAMERA_NUM][3];
	double t2[CAMERA_NUM][3];
	double R1[CAMERA_NUM][3];
	double t1[CAMERA_NUM][3];
	double d_distortion[CAMERA_NUM][5];

	//重投影优化K, camera_pos, pose_timestamp
	ceres::Problem reprojection_problem;
	for (int i = 0; i < CAMERA_NUM; i++){
		camera_2_array(intrinsic[i], K[i]);
		transform2RT(camera_pos[i], R2[i], t2[i]);
		transform2RT(pose_timestamp[i], R1[i], t1[i]);
		matrix_2_array(distortion[i], d_distortion[i], 1, 5);
	}
	for (int i = 0; i < all_observe.size(); i++) {
		for (int j = 0; j < all_observe[i].size(); j++) {
			for (int k = 0; k < all_observe[i][j].size(); k++) {
				ceres::CostFunction * reprojection_cost_function = ReprojectionError::Create(all_observe[i][j][k], object[k]);
				reprojection_problem.AddResidualBlock(
					reprojection_cost_function, NULL, K[i], R2[i], t2[i], R1[j], t1[j], d_distortion[i]);

			}
		}

	}
	ceres::Solver::Options reprojection_options;
	reprojection_options.linear_solver_type = ceres::DENSE_SCHUR;
	reprojection_options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(reprojection_options, &reprojection_problem, &reprojection_summary);
	fcout << "\n\n" << reprojection_summary.FullReport() << "\n";

	//double数组转化为矩阵
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();
	for (int i = 0; i < CAMERA_NUM; i++) {
		Mat_<float> t_intrinsic(intrinsic[i].rows, intrinsic[i].cols);
		Mat_<float> t_camera_pos(camera_pos[i].rows, camera_pos[i].cols);
		Mat_<float> t_pose_timestamp(pose_timestamp[i].rows, pose_timestamp[i].cols);
		Mat_<float> t_distortion(distortion[i].rows, distortion[i].cols);

		array_2_camera(K[i], t_intrinsic);
		RT2transform(R2[i], t2[i], t_camera_pos);
		RT2transform(R1[i], t1[i], t_pose_timestamp);
		array_2_matrix(t_distortion, d_distortion[i], 1, 5);

		optimized_distortion.push_back(t_distortion);
		optimized_intrinsic.push_back(t_intrinsic);
		optimized_camera_pos.push_back(t_camera_pos);
		optimized_pose_timestamp.push_back(t_pose_timestamp);
	}

	output_all_point_and_loss(all_observe, optimized_intrinsic, optimized_camera_pos,
		optimized_pose_timestamp, optimized_distortion, recon_pts, object,
		"", "", OUT_PATH + "reprojection_optimize重投影优化后____重投影（由物体坐标点到像素点）.txt"); // after reprojection optimizing

	writeParams(OUT_PATH + "reprojection_intrinsic.xml", optimized_intrinsic, "K_", optimized_distortion, "DISTORTION_");
	writeParams(OUT_PATH + "reprojection_extrinsic_results.xml", optimized_camera_pos, "camera_pose_", optimized_pose_timestamp, "pose_timestamp_");
}

void reconstruction_optimize(const vector < vector < vector < Point3f > > > & all_observe, const vector<Point3d> & object)
{
	vector< Mat_<float> >camera_pos, pose_timestamp, distortion, intrinsic;
	vector< Mat_<float> >optimized_camera_pos, optimized_pose_timestamp, optimized_distortion, optimized_intrinsic;
	camera_pos.clear(), pose_timestamp.clear(), distortion, intrinsic.clear();
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();

	//读入参数
	readParams(IN_PATH + "intrinsic.xml", intrinsic, Size(3, 3), "K_", distortion, Size(1, 5), "DISTORTION_");
	readParams(IN_PATH + "exito0.xml", camera_pos, Size(4, 4), "camera_pose_", camera_pos);
	readParams(IN_PATH + "ex0.xml", pose_timestamp, Size(4, 4), "pose_timestamp_", pose_timestamp);
	
	vector< vector< vector <Point3d> > > recon_pts;
	vector< vector< vector < vector <Point3d> > > > v4_pts;
	recon_pts.clear(), v4_pts.clear();
	//求出si
	for (int i = 0; i < all_observe.size(); i++) //(0,1),(0,2),...,(0,10);(1,1),(1,2)...,(1,10);...;(9,10);
	{
		vector< vector<Point3d> > v3pts;
		v3pts.clear();

		Mat K0 = intrinsic[i];

		for (int j = 0; j < all_observe[i].size(); j++) {

			vector< vector<Point3d> > v2pts;
			v2pts.clear();
			for (int k = 0; k < CAMERA_NUM; k++){
				if (k == i)continue;

				Mat K1 = intrinsic[k];
				vector<Point3d> tmp_pts;
				tmp_pts.clear();

				Mat R = camera_pos[k] * camera_pos[i].inv();
				//最小二乘三角测量
				triangulation(tmp_pts, all_observe[i][j], all_observe[k][j], K0, K1, R);
				v2pts.push_back(tmp_pts);
			}
			v3pts.push_back(optimize_array(v2pts));
		}
		recon_pts.push_back(v3pts);
	}

	output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, recon_pts, object,
		OUT_PATH + "reconstruction_optimize优化前__反向投影点云____________________.txt",
		OUT_PATH + "reconstruction_optimize优化前__反向投影（由像素点到物体坐标点）.txt",
		OUT_PATH + "reconstruction_optimize优化前____重投影（由物体坐标点到像素点）.txt"); // before optimizating

	output_image_point_cloud(all_observe, intrinsic, camera_pos, pose_timestamp, recon_pts, OUT_PATH + "reconstruction_optimize图像点云"); // before optimizating

	//optimize_all_observe(all_observe, intrinsic, camera_pos, pose_timestamp, recon_pts, object,board_size); // before optimizating
	//output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, recon_pts, object,
	//	OUT_PATH + "去掉误差较大的点后__反向投影点云____________________.txt",
	//	OUT_PATH + "去掉误差较大的点后__反向投影（由像素点到物体坐标点）.txt",
	//	""); // before optimizating

	double K[CAMERA_NUM][4];
	double R2[CAMERA_NUM][3];
	double t2[CAMERA_NUM][3];
	double R1[CAMERA_NUM][3];
	double t1[CAMERA_NUM][3];
	double d_distortion[CAMERA_NUM][5];

	////反向投影优化K, camera_pos, pose_timestamp
	ceres::Problem reverse_projection_problem;	//2880
	for (int i = 0; i < CAMERA_NUM; i++){
		camera_2_array(intrinsic[i], K[i]);
		transform2RT(camera_pos[i], R2[i], t2[i]);
		transform2RT(pose_timestamp[i], R1[i], t1[i]);
		matrix_2_array(distortion[i], d_distortion[i], 1, 5);
	}
	for (int i = 0; i < all_observe.size(); i++){
		for (int j = 0; j < all_observe[i].size(); j++){
			if (recon_pts[i][j].size() > 0){
				for (int k = 0; k < all_observe[i][j].size(); k++){
					bool flag = all_observe[i][j][k].z;//是否参与计算
					if (flag){
						ceres::CostFunction * cost_function = ReconstrutionError::Create(all_observe[i][j][k], object[k], recon_pts[i][j][k]);
						reverse_projection_problem.AddResidualBlock(cost_function, NULL, K[i], R2[i], t2[i], R1[j], t1[j], d_distortion[i]);
					}
				}
			}
		}
	}

	ceres::Solver::Options reverse_projection_options;
	reverse_projection_options.linear_solver_type = ceres::DENSE_SCHUR;
	reverse_projection_options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary reverse_projection_summary;
	ceres::Solve(reverse_projection_options, &reverse_projection_problem, &reverse_projection_summary);
	fcout << "\n\n" << reverse_projection_summary.FullReport() << "\n";

	//double数组转化为矩阵
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();
	for (int i = 0; i < CAMERA_NUM; i++) {
		Mat_<float> t_intrinsic(intrinsic[i].rows, intrinsic[i].cols);
		Mat_<float> t_camera_pos(camera_pos[i].rows, camera_pos[i].cols);
		Mat_<float> t_pose_timestamp(pose_timestamp[i].rows, pose_timestamp[i].cols);
		Mat_<float> t_distortion(distortion[i].rows, distortion[i].cols);

		array_2_camera(K[i], t_intrinsic);
		RT2transform(R2[i], t2[i], t_camera_pos);
		RT2transform(R1[i], t1[i], t_pose_timestamp);
		array_2_matrix(t_distortion, d_distortion[i], 1, 5);

		optimized_distortion.push_back(t_distortion);
		optimized_intrinsic.push_back(t_intrinsic);
		optimized_camera_pos.push_back(t_camera_pos);
		optimized_pose_timestamp.push_back(t_pose_timestamp);
	}

	output_all_point_and_loss(all_observe, optimized_intrinsic, optimized_camera_pos,
		optimized_pose_timestamp, optimized_distortion, recon_pts, object,
		OUT_PATH + "reconstruction_optimize反向投影优化后__反向投影点云____________________.txt",
		OUT_PATH + "reconstruction_optimize反向投影优化后__反向投影（由像素点到物体坐标点）.txt", ""); // before optimizating

	writeParams(OUT_PATH + "reconstruction_intrinsic.xml", optimized_intrinsic, "K_", optimized_distortion, "DISTORTION_");
	writeParams(OUT_PATH + "reconstruction_extrinsic_results.xml", optimized_camera_pos, "camera_pose_", optimized_pose_timestamp, "pose_timestamp_");
}

int main(int argc, char * argv[])
{
	cout << "calculating..." << endl;
	fcout.open(OUT_PATH + "console__report.txt", ios::out);

	//读入棋盘格所有的角点
	vector < vector < vector < Point3f > > > all_observe; //z用作是否参与运算的flag
	all_observe.clear();
	get_all_corner_point(all_observe, board_size);
	out_all_point(all_observe);
	vector<Point3d> object;
	object.clear();
	double squareSize = 50;
	calcChessboardCorners(board_size, squareSize, object);
	
	reprojection_optimize(all_observe, object);
	reconstruction_optimize(all_observe, object);

	fcout.close();
	system("pause");

	return 0;
}






void get_all_corner_point(vector < vector < vector < Point3f > > > & all_observe, Size board_size)
{
#ifdef _DEBUG
	_D_VIEW.clear();
#endif

	all_observe.clear();
	for (int _i = 0; _i < CAMERA_NUM; _i++)
	{
		vector<Mat> vec_image_i = readImage(IN_PATH, MAX_IMAGE_NUM, _i);

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
	const vector<vector<vector<Point3d> > > recon_pts, const vector<Point3d> object,
	const string ss1,
	const string ss2,
	const string ss3,
	const bool is_have_si
	)
{
	fstream fs1(ss1, ios::out);
	fstream fs2(ss2, ios::out);
	fstream fs3(ss3, ios::out);

	fstream fs_error(ss2 + "error.txt", ios::out);

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
					if (recon_pts[Camera_i][image_j].size() <= 0) {
						continue;
					}
				}
				bool flag = all_observe[Camera_i][image_j][k].z;//是否参与计算
				if (flag) {
					Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					tmp_image_point.at<float>(0, 0) = recon_pts[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].x;
					tmp_image_point.at<float>(1, 0) = recon_pts[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].y;
					tmp_image_point.at<float>(2, 0) = recon_pts[Camera_i][image_j][k].z;
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

					double tmp_loss = 0.5 * pow(object_point.x - object[k].x, 2) + 0.5 * pow(object_point.y - object[k].y, 2) + 0.5 * pow(object_point.z - object[k].z, 2);
					fs_error << tmp_loss << endl;

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


void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d> & corners)
{
	corners.resize(0);
	for (int i = 0; i < boardSize.height; i++) {     //height和width位置不能颠倒
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(Point3d(j*squareSize, i*squareSize, 0));
		}
	}
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

bool triangulation(vector<Point3d> & pts, const vector<Point3f> & observe0, const vector<Point3f> & observe1,
	const Mat & K0, const Mat & K1,
	const Mat & R
	)
{
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
	cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);
	//vector<Point3d> points0,points1;
	for (int i = 0; i < pts_4d.cols; i++){
		Mat x = pts_4d.col(i);
		x /= x.at<double>(3, 0);
		Point3d p(
			x.at<double>(0, 0),
			x.at<double>(1, 0),
			x.at<double>(2, 0)
			);
		pts.push_back(p);
		/*Point3d p1 = cam1_to_cam2(p, T2);
		points0.push_back(p);
		points1.push_back(p1);*/
	}

	return true;
}
