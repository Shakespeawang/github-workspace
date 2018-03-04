

#include <fstream>
#include <string>
#include <opencv.hpp>
#include <ceres.h>
#include <math.h>

using namespace cv;
using namespace ceres;
using namespace std;


const Size board_size = Size(9, 6);
const string IN_PATH = "D:\\毕业设计\\图漾第二次标定数据\\";
const string OUT_PATH = "D:\\毕业设计\\图漾第二次标定数据\\out\\";
const int CAMERA_NUM = 11;
const int MAX_IMAGE_NUM = 11;
vector< vector<Mat>> _D_VIEW;

double optimize_array(vector<double> arr);
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
bool get_s0_s1(vector<double> & s_0, vector<double> & s_1, vector<Point3f> observe0, vector<Point3f> observe1, Mat K0, Mat K1, Mat camera_pos);
void get_all_corner_point(vector < vector < vector < Point3f > > > & all_observe, Size board_size);
void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d> & corners);

bool readParams(string path, vector<Mat_<float>> & vec1, Size size1, string flag1, vector<Mat_<float>> & vec2, Size size2, string flag2 = NULL, int n = 11);
bool writeParams(string path, vector<Mat_<float>> & vec1, string flag1, vector<Mat_<float>> & vec2, string flag2);

vector<Mat> readImage(string path, int _k, int _j, int _i = 0, string suffix = ".png", string connector = "-");
Mat readImage(int _i, int _j, int _k = 0, string path = IN_PATH, string suffix = ".png", string connector = "-");


bool writeImage(Mat image, int _i, int _j, int _k = 0, string path = OUT_PATH, string suffix = ".png", string connector = "-");

bool matrix_2_array(const Mat_<float> mat, double arr[], const int arr_row, const int arr_col);
bool array_2_matrix(Mat_<float> & mat, const double arr[], const int arr_row, const int arr_col);

bool out_all_point(const vector < vector < vector < Point3f > > > & all_observe, const string ss1 = OUT_PATH + "棋盘格角点.txt");

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


struct ReverseProjectionError {
	ReverseProjectionError(Point3d observe, Point3d objection, double si)
	: observe_(observe), objection_(objection), si_(si) {}

	template <typename T>
	bool operator()(const T * const intrinsic, const T * const camera_pos, const T * const pose_timestamp, T * residual) const
	{
		T homo1[4 * 4 + 0], homo2[4 * 4 + 0], homo_inv[4 * 4 + 0], re_intrinsic[4 * 4];

		matrix_resize(intrinsic, re_intrinsic, 3, 3, 4, 4);
		matrix_multiply(re_intrinsic, camera_pos, homo1, 4, 4, 4, 4);
		matrix_multiply(homo1, pose_timestamp, homo2, 4, 4, 4, 4);

		matrix_inv(homo2, homo_inv, 4, 4);

		T tmp_image_point[4 * 1];
		tmp_image_point[0 + 0 * 4] = T(si_ * observe_.x);
		tmp_image_point[1 + 0 * 4] = T(si_ * observe_.y);
		tmp_image_point[2 + 0 * 4] = T(si_);
		tmp_image_point[3 + 0 * 4] = T(1.0);

		T possible_objection_point[4 * 1 + 0];
		matrix_multiply(homo_inv, tmp_image_point, possible_objection_point, 4, 4, 4, 1);

		residual[0] = (T)possible_objection_point[0] - T(objection_.x);
		residual[1] = (T)possible_objection_point[1] - T(objection_.y);
		residual[2] = (T)possible_objection_point[2] - T(objection_.z);

		return true;
	}

	static ceres::CostFunction * Create(Point3d observe, Point3d objection, double si) {

		return (new ceres::AutoDiffCostFunction<ReverseProjectionError, 3, 9, 16, 16>
			(new ReverseProjectionError(observe, objection, si)));

	}
	double si_;
	Point3d observe_;
	Point3d objection_;
};

struct ReprojectionError {
	ReprojectionError(Point3d observe, Point3d objection)
	: observe_(observe), objection_(objection) {}

	template <typename T>
	bool operator()(const T * const intrinsic, const T * const camera_pos, const T * const pose_timestamp, const T * const distortion, T * residual) const
	{
		T homo[4 * 4 + 0];
		matrix_multiply(camera_pos, pose_timestamp, homo, 4, 4, 4, 4);

		T tmp_image_point[4 * 1];
		tmp_image_point[0 + 0 * 4] = T(objection_.x);
		tmp_image_point[1 + 0 * 4] = T(objection_.y);
		tmp_image_point[2 + 0 * 4] = T(objection_.z);
		tmp_image_point[3 + 0 * 4] = T(1.0);

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

		T fx = intrinsic[0], fy = intrinsic[4], cx = intrinsic[2], cy = intrinsic[5];

		T predicited_x = fx * x_distorted + cx;
		T predicited_y = fy * y_distorted + cy;

		residual[0] = (T)predicited_x - T(observe_.x);
		residual[1] = (T)predicited_y - T(observe_.y);

		return true;
	}

	static ceres::CostFunction * Create(Point3d observe, Point3d objection) {

		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 16, 16, 5>
			(new ReprojectionError(observe, objection)));

	}
	Point3d observe_;
	Point3d objection_;
};

struct XOYDistanceError {
	XOYDistanceError(Point3d observe, Point3d objection, double si)
	: observe_(observe), objection_(objection), si_(si) {}

	template <typename T>
	bool operator()(const T * const intrinsic, const T * const camera_pos, const T * const pose_timestamp, T * residual) const
	{
		T homo1[4 * 4 + 0], homo2[4 * 4 + 0], homo_inv[4 * 4 + 0], re_intrinsic[4 * 4];

		matrix_resize(intrinsic, re_intrinsic, 3, 3, 4, 4);
		matrix_multiply(re_intrinsic, camera_pos, homo1, 4, 4, 4, 4);
		matrix_multiply(homo1, pose_timestamp, homo2, 4, 4, 4, 4);

		matrix_inv(homo2, homo_inv, 4, 4);

		T tmp_image_point[4 * 1];
		tmp_image_point[0 + 0 * 4] = T(si_ * observe_.x);
		tmp_image_point[1 + 0 * 4] = T(si_ * observe_.y);
		tmp_image_point[2 + 0 * 4] = T(si_);
		tmp_image_point[3 + 0 * 4] = T(1.0);

		T possible_objection_point[4 * 1 + 0];
		matrix_multiply(homo_inv, tmp_image_point, possible_objection_point, 4, 4, 4, 1);

		residual[0] = (T)possible_objection_point[2] - T(objection_.z);

		return true;
	}

	static ceres::CostFunction * Create(Point3d observe, Point3d objection, double si) {

		return (new ceres::AutoDiffCostFunction<XOYDistanceError, 1, 9, 16, 16>
			(new XOYDistanceError(observe, objection, si)));

	}
	double si_;
	Point3d observe_;
	Point3d objection_;
};



int main(int argc, char * argv[])
{

	fstream fcout(OUT_PATH + "console__report.txt", ios::out);


	vector< Mat_<float> >camera_pos, pose_timestamp, distortion, intrinsic;
	vector< Mat_<float> >optimized_camera_pos, optimized_pose_timestamp, optimized_distortion, optimized_intrinsic;
	camera_pos.clear(), pose_timestamp.clear(), distortion, intrinsic.clear();
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_distortion.clear(), optimized_intrinsic.clear();

	//读入参数
	readParams(IN_PATH + "cameras_intrinsic_parameters1.xml",
		intrinsic, Size(3, 3), "K_", distortion, Size(1, 5), "DISTORTION_");
	readParams(IN_PATH + "multi-extrinsic-results1.xml",
		camera_pos, Size(4, 4), "camera_pose_", pose_timestamp, Size(4, 4), "pose_timestamp_");

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




	optimize_all_observe(all_observe, intrinsic, camera_pos, pose_timestamp, S_Si, object); // before optimizating
	
	output_all_point_and_loss(all_observe, intrinsic, camera_pos, pose_timestamp, distortion, S_Si, object,
		OUT_PATH + "去掉误差较大的点后__反向投影点云____________________.txt",
		OUT_PATH + "去掉误差较大的点后__反向投影（由像素点到物体坐标点）.txt",
		""); // before optimizating



	double d_intrinsic[CAMERA_NUM][3 * 3];
	double d_camera_pos[CAMERA_NUM][4 * 4];
	double d_pose_timestamp[CAMERA_NUM][4 * 4];
	double d_distortion[CAMERA_NUM][5];

	//重投影优化K, camera_pos, pose_timestamp
	ceres::Problem reprojection_problem;

	for (int camera_i = 0; camera_i < all_observe.size(); camera_i++) {

		matrix_2_array(intrinsic[camera_i], d_intrinsic[camera_i], 3, 3);
		matrix_2_array(camera_pos[camera_i], d_camera_pos[camera_i], 4, 4);
		matrix_2_array(distortion[camera_i], d_distortion[camera_i], 1, 5);

		for (int image_j = 0; image_j < all_observe[camera_i].size(); image_j++) {

			matrix_2_array(pose_timestamp[image_j], d_pose_timestamp[image_j], 4, 4);

			for (int k = 0; k < all_observe[camera_i][image_j].size(); k++) {
				ceres::CostFunction * reprojection_cost_function = ReprojectionError::Create(all_observe[camera_i][image_j][k], object[k]);
				reprojection_problem.AddResidualBlock(
					reprojection_cost_function, NULL,
					d_intrinsic[camera_i], d_camera_pos[camera_i], d_pose_timestamp[image_j], d_distortion[camera_i]);

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
		array_2_matrix(t_intrinsic, d_intrinsic[i], 3, 3);
		array_2_matrix(t_camera_pos, d_camera_pos[i], 4, 4);
		array_2_matrix(t_pose_timestamp, d_pose_timestamp[i], 4, 4);
		array_2_matrix(t_distortion, d_distortion[i], 1, 5);

		optimized_distortion.push_back(t_distortion);
		optimized_intrinsic.push_back(t_intrinsic);
		optimized_camera_pos.push_back(t_camera_pos);
		optimized_pose_timestamp.push_back(t_pose_timestamp);
	}

	output_all_point_and_loss(all_observe, optimized_intrinsic, optimized_camera_pos,
		optimized_pose_timestamp, optimized_distortion, S_Si, object,
		"", "", OUT_PATH + "重投影优化后____重投影（由物体坐标点到像素点）.txt"); // after reprojection optimizing

	writeParams(OUT_PATH + "reprojection_cameras_intrinsic_parameters.xml", optimized_intrinsic, "K_", optimized_distortion, "DISTORTION_");
	writeParams(OUT_PATH + "reprojection_multi-extrinsic-results.xml", optimized_camera_pos, "camera_pose_", optimized_pose_timestamp, "pose_timestamp_");



	//反向投影优化,误差取到xoy平面的距离，K, camera_pos, pose_timestamp
	ceres::Problem xoy_distance_problem;	//2880
	for (int camera_i = 0; camera_i < all_observe.size(); camera_i++) {

		matrix_2_array(intrinsic[camera_i], d_intrinsic[camera_i], 3, 3);
		matrix_2_array(camera_pos[camera_i], d_camera_pos[camera_i], 4, 4);

		for (int image_j = 0; image_j < all_observe[camera_i].size(); image_j++) {

			matrix_2_array(pose_timestamp[image_j], d_pose_timestamp[image_j], 4, 4);

			if (S_Si[camera_i][image_j].size() > 0) {
				for (int k = 0; k < all_observe[camera_i][image_j].size(); k++) {
					bool flag = all_observe[camera_i][image_j][k].z;//是否参与计算
					if (flag) {
						ceres::CostFunction * cost_function = XOYDistanceError::Create(all_observe[camera_i][image_j][k], object[k], S_Si[camera_i][image_j][k]);
						xoy_distance_problem.AddResidualBlock(
							cost_function, NULL,
							d_intrinsic[camera_i], d_camera_pos[camera_i], d_pose_timestamp[image_j]);
					}
				}
			}
		}
	}

	ceres::Solver::Options xoy_distance_options;
	xoy_distance_options.linear_solver_type = ceres::DENSE_SCHUR;
	xoy_distance_options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary xoy_distance_summary;
	ceres::Solve(xoy_distance_options, &xoy_distance_problem, &xoy_distance_summary);
	fcout << "\n\n" << xoy_distance_summary.FullReport() << "\n";

	//double数组转化为矩阵
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_intrinsic.clear();
	for (int i = 0; i < CAMERA_NUM; i++) {
		Mat_<float> t_intrinsic(intrinsic[i].rows, intrinsic[i].cols);
		Mat_<float> t_camera_pos(camera_pos[i].rows, camera_pos[i].cols);
		Mat_<float> t_pose_timestamp(pose_timestamp[i].rows, pose_timestamp[i].cols);
		array_2_matrix(t_intrinsic, d_intrinsic[i], 3, 3);
		array_2_matrix(t_camera_pos, d_camera_pos[i], 4, 4);
		array_2_matrix(t_pose_timestamp, d_pose_timestamp[i], 4, 4);

		//optimized_distortion.push_back(distortion[i]);
		optimized_intrinsic.push_back(t_intrinsic);
		optimized_camera_pos.push_back(t_camera_pos);
		optimized_pose_timestamp.push_back(t_pose_timestamp);
	}

	output_all_point_and_loss(all_observe, optimized_intrinsic, optimized_camera_pos,
		optimized_pose_timestamp, optimized_distortion, S_Si, object,
		OUT_PATH + "误差取到xoy平面的距离__反向投影优化后__反向投影点云____________________.txt",
		OUT_PATH + "误差取到xoy平面的距离__反向投影优化后__反向投影（由像素点到物体坐标点）.txt", ""); // before optimizating

	writeParams(OUT_PATH + "xoy_distance_summary_cameras_intrinsic_parameters.xml", optimized_intrinsic, "K_", optimized_distortion, "DISTORTION_");
	writeParams(OUT_PATH + "xoy_distance_summary_multi-extrinsic-results.xml", optimized_camera_pos, "camera_pose_", optimized_pose_timestamp, "pose_timestamp_");


	////反向投影优化K, camera_pos, pose_timestamp
	ceres::Problem reverse_projection_problem;	//2880
	for (int camera_i = 0; camera_i < all_observe.size(); camera_i++){

		matrix_2_array(intrinsic[camera_i], d_intrinsic[camera_i], 3, 3);
		matrix_2_array(camera_pos[camera_i], d_camera_pos[camera_i], 4, 4);

		for (int image_j = 0; image_j < all_observe[camera_i].size(); image_j++){

			matrix_2_array(pose_timestamp[image_j], d_pose_timestamp[image_j], 4, 4);

			if (S_Si[camera_i][image_j].size() > 0){
				for (int k = 0; k < all_observe[camera_i][image_j].size(); k++){
					bool flag = all_observe[camera_i][image_j][k].z;//是否参与计算
					if (flag){
						ceres::CostFunction * cost_function = ReverseProjectionError::Create(all_observe[camera_i][image_j][k], object[k], S_Si[camera_i][image_j][k]);
						reverse_projection_problem.AddResidualBlock(
							cost_function, NULL,
							d_intrinsic[camera_i], d_camera_pos[camera_i], d_pose_timestamp[image_j]);
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
	optimized_camera_pos.clear(), optimized_pose_timestamp.clear(), optimized_intrinsic.clear();
	for (int i = 0; i < CAMERA_NUM; i++) {
		Mat_<float> t_intrinsic(intrinsic[i].rows, intrinsic[i].cols);
		Mat_<float> t_camera_pos(camera_pos[i].rows, camera_pos[i].cols);
		Mat_<float> t_pose_timestamp(pose_timestamp[i].rows, pose_timestamp[i].cols);
		array_2_matrix(t_intrinsic, d_intrinsic[i], 3, 3);
		array_2_matrix(t_camera_pos, d_camera_pos[i], 4, 4);
		array_2_matrix(t_pose_timestamp, d_pose_timestamp[i], 4, 4);

		//optimized_distortion.push_back(distortion[i]);
		optimized_intrinsic.push_back(t_intrinsic);
		optimized_camera_pos.push_back(t_camera_pos);
		optimized_pose_timestamp.push_back(t_pose_timestamp);
	}

	output_all_point_and_loss(all_observe, optimized_intrinsic, optimized_camera_pos,
		optimized_pose_timestamp, optimized_distortion, S_Si, object,
		OUT_PATH + "反向投影优化后__反向投影点云____________________.txt",
		OUT_PATH + "反向投影优化后__反向投影（由像素点到物体坐标点）.txt", ""); // before optimizating

	writeParams(OUT_PATH + "reverse_projection_cameras_intrinsic_parameters.xml", optimized_intrinsic, "K_", optimized_distortion, "DISTORTION_");
	writeParams(OUT_PATH + "reverse_projection_multi-extrinsic-results.xml", optimized_camera_pos, "camera_pose_", optimized_pose_timestamp, "pose_timestamp_");



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


bool matrix_2_array(const Mat_<float> mat, double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < arr_row; i++) {
		for (int j = 0; j < arr_col; j++) {
			if (i * arr_col + j < arr_row * arr_col) {
				if (i < mat.rows && j < mat.cols)
					arr[i * arr_col + j] = mat.at<float>(i, j);
				else if (i == j)
					arr[i * arr_col + j] = 1.0;
				else
					arr[i * arr_col + j] = 0.0;
			}
			//cout << arr[i * arr_col + j] << "\t";
		}
		//cout << "\n";
	}
	return true;
}
bool array_2_matrix(Mat_<float> & mat, const double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			if (i < mat.rows && j < mat.cols) {
				if (i * arr_col + j < arr_row * arr_col)
					mat.at<float>(i, j) = arr[i * arr_col + j];
				else if (i == j)
					mat.at<float>(i, j) = 1.0;
				else
					mat.at<float>(i, j) = 0.0;
			}
			//cout << mat.at<float>(i, j) << "\t";
		}
		//cout << "\n";
	}
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

bool writeParams(string path, vector<Mat_<float>> & vec1, string flag1, vector<Mat_<float>> & vec2, string flag2)
{
	FileStorage fs(path, FileStorage::WRITE);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < vec1.size(); i++)
	{
		stringstream ss1;
		ss1 << flag1 << i;
		fs << ss1.str() << vec1[i];
	}
	for (int i = 0; i < vec2.size(); i++)
	{
		stringstream ss2;
		ss2 << flag2 << i;
		fs << ss2.str() << vec2[i];
	}
	fs.release();
	return true;
}

bool readParams(string path, vector<Mat_<float>> & vec1, Size size1, string flag1, vector<Mat_<float>> & vec2, Size size2, string flag2, int n)
{
	vec1.clear();
	vec2.clear();
	FileStorage fs(path, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < n; i++)
	{
		stringstream ss1, ss2;
		ss1 << flag1 << i;
		ss2 << flag2 << i;
		Mat mat1(size1, CV_32FC1);
		Mat mat2(size2, CV_32FC1);
		fs[ss1.str()] >> mat1;
		fs[ss2.str()] >> mat2;
		vec1.push_back(mat1);
		vec2.push_back(mat2);
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
	s_0.clear(),s_1.clear();
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
	vector<Point3d> points0,points1;
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


bool get_s0_s1(vector<double> & s_0, vector<double> & s_1,
	vector<Point3f> observe0, vector<Point3f> observe1,
	Mat K0, Mat K1,
	Mat camera_pos
	)
{
	s_0.clear();
	s_1.clear();
	if (observe0.size() != observe1.size() || observe0.size() <= 0 || observe1.size() <= 0)
		return false;
	for (int j = 0; j < observe0.size(); j++) {
		Mat position0(3, 1, CV_32FC1);
		Mat observe0_point(3, 1, CV_32FC1);
		observe0_point.at<float>(0, 0) = observe0[j].x;
		observe0_point.at<float>(1, 0) = observe0[j].y;
		observe0_point.at<float>(2, 0) = 1.0f;
		position0 = K0.inv() * observe0_point;

		Mat position1(3, 1, CV_32FC1);
		Mat observe1_point(3, 1, CV_32FC1);
		observe1_point.at<float>(0, 0) = observe1[j].x;
		observe1_point.at<float>(1, 0) = observe1[j].y;
		observe1_point.at<float>(2, 0) = 1.0f;
		position1 = K1.inv() * observe1_point;

		double s0, s1, s01, s02, s11, s12;
		double x0 = position0.at<float>(0, 0);
		double y0 = position0.at<float>(1, 0);
		double z0 = position0.at<float>(2, 0);
		double x1 = position1.at<float>(0, 0);
		double y1 = position1.at<float>(1, 0);
		double z1 = position1.at<float>(1, 0);

		float **arr = new float *[camera_pos.rows];
		for (int j = 0; j < camera_pos.rows; ++j)
			arr[j] = camera_pos.ptr<float>(j);

#ifdef _DEBUG
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				//fcout << arr[i][j] << "\t";
			}
			//fcout << endl;
		}
		Mat _D_01 = camera_pos;
#endif // _DEBUG


		double alpha = arr[0][0] * x0 + arr[0][1] * y0 + arr[0][2] * z0;
		double beta = arr[1][0] * x0 + arr[1][1] * y0 + arr[1][2] * z0;
		double gama = arr[2][0] * x0 + arr[2][1] * y0 + arr[2][2] * z0;
		try {
			s0 = (x1 / y1 * arr[1][3] - arr[0][3]) / (alpha - beta * x1 / y1);
			s1 = s0 * gama + arr[2][3];
		}
		catch (exception e) {
			continue;
		}
		s_0.push_back(s0);
		s_1.push_back(s1);
	}
	return true;
}
double optimize_array(vector<double> arr) {

	double ret_s0 = 0.0;
	if (arr.size() <= 0)
		return ret_s0;
	sort(arr.begin(), arr.end());


	int cnt = 0;
	for (int i = 0; i < arr.size(); i++) {
		if (arr[i] > 0 || arr[i] < 0) {
			cnt++;
			ret_s0 += arr[i];
		}
	}
	if (cnt <= 0)
		return ret_s0;
	else
		return ret_s0 / cnt;
}

//
//int Camera_i = 0;
//
//for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++){
//
//	stringstream ss;
//	ss << IN_PATH + "points-at-timestamp-" << image_j << ".txt";
//	fstream fs(ss.str(), ios::out);
//
//	for (Camera_i = 0; Camera_i < all_observe.size(); Camera_i++){
//
//		Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
//		for (int i = 0; i < 4; i++){
//			float * ptr = tmp_intrinsic_i.ptr<float>(i);
//			for (int j = 0; j < 4; j++){
//				if (i < 3 && j < 3)
//					ptr[j] = intrinsic[Camera_i].at<float>(i, j);
//				else if (3 == i && 3 == j)
//					ptr[j] = 1.0;
//				else
//					ptr[j] = 0.0;
//			}
//		}
//#ifdef _DEBUG
//		Mat _D_02 = pose_timestamp[Camera_i];
//#endif // _DEBUG
//
//		Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
//		homo = tmp_intrinsic_i * pose_timestamp[Camera_i];
//		Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
//		homo_inv = homo.inv();
//
//		vector<Point3d> vec;
//		vec.clear();
//		for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++){
//			Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
//			tmp_image_point.at<float>(0, 0) = S_Si[Camera_i] * all_observe[Camera_i][image_j][k].x;
//			tmp_image_point.at<float>(1, 0) = S_Si[Camera_i] * all_observe[Camera_i][image_j][k].y;
//			tmp_image_point.at<float>(2, 0) = S_Si[Camera_i];
//			tmp_image_point.at<float>(3, 0) = 1.0;
//
//#ifdef _DEBUG
//			Mat _D_03 = _D_VIEW[Camera_i][image_j];
//#endif // _DEBUG
//
//
//			Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
//			possible_object_point = homo_inv * tmp_image_point;
//
//
//			Mat possible_object_point_at_camera0(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
//			possible_object_point_at_camera0 = camera_pos[Camera_i].inv() * possible_object_point;
//
//			Point3d object_point;
//			object_point.x = possible_object_point_at_camera0.at<float>(0, 0);
//			object_point.y = possible_object_point_at_camera0.at<float>(1, 0);
//			object_point.z = possible_object_point_at_camera0.at<float>(2, 0);
//
//			fs << object_point.x << " " << object_point.y << " " << object_point.z << endl;
//
//			vec.push_back(object_point);
//		}
//	}
//	fs.close();
//	Camera_i = 0;
//}

