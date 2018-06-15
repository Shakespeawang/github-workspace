#include "binocularCalibration.h"



void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	std::vector<cv::DMatch> matches;

	//-- 初始化
	cv::Mat descriptors_1, descriptors_2;
	// used in OpenCV3 
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
	cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
	// use this if you are in OpenCV2 
	// Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
	// Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	//-- 第一步:检测 Oriented FAST 角点位置
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);

	//-- 第二步:根据角点位置计算 BRIEF 描述子
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	//-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
	std::vector<cv::DMatch> match;
	//BFMatcher matcher ( NORM_HAMMING );
	matcher->match(descriptors_1, descriptors_2, match);

	//-- 第四步:匹配点对筛选
	double min_dist = 10000, max_dist = 0;

	//找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = match[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (match[i].distance <= std::max(2 * min_dist, 30.0))
		{
			matches.push_back(match[i]);
		}
	}

	//-- 把匹配点转换为vector<cv::Point2f>的形式

	for (int i = 0; i < (int)matches.size(); i++)
	{
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].trainIdx].pt);
	}
}


cv::Point2d pixel2cam(const cv::Point2d&p, const cv::Mat& K)
{
	return cv::Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}


void pose_estimation_2d2d(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, double focal_length, cv::Point2d principal_point, cv::Mat& R, cv::Mat& t)
{
	//-- 计算基础矩阵
	cv::Mat fundamental_matrix;
	cv::Mat essential_matrix;
	fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);

	//-- 计算本质矩阵
	essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);

	//-- 从本质矩阵中恢复旋转和平移信息.
	recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}


void pose_estimate(const cv::Mat& img_1,const cv::Mat& img_2,const cv::Mat& K, cv::Mat& R,cv::Mat& t)
{
	std::vector<cv::Point2f> keypoints_1, keypoints_2;
	find_feature_matches(img_1, img_2, keypoints_1, keypoints_2);

	cv::Point2d principal_point(K.at<double>(0,2), K.at<double>(1, 2));	//相机光心, TUM dataset标定值
	double focal_length = 0.5 * K.at<double>(0, 0) + 0.5 * K.at<double>(1, 1);			//相机焦距, TUM dataset标定值
										//-- 估计两张图像间运动
	pose_estimation_2d2d(keypoints_1, keypoints_2, focal_length, principal_point, R, t);

	//epipolarOptimize(keypoints_1, keypoints_2, K, K, R, t);
}
















/*
*	@ 2.4)
*/
template<typename T>
void CalcRodrigues(const T a[3], T b[9])
{
	if (a[0] == (T)0.0&&a[1] == (T)0.0&&a[2] == (T)0.0)
	{
		b[0] = (T)1.0;		b[1] = (T)0.0;		b[2] = (T)0.0;
		b[3] = (T)0.0;		b[4] = (T)1.0;		b[5] = (T)0.0;
		b[6] = (T)0.0;		b[7] = (T)0.0;		b[8] = (T)1.0;
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
void RT2TransformTemplate(const T R[3], const T t[3], T Transform[])
{
	T mR[9];
	CalcRodrigues(R, mR);
	matrix_resize(mR, Transform, 3, 3, 3, 4);
	Transform[0 * 4 + 3] = t[0];
	Transform[1 * 4 + 3] = t[1];
	Transform[2 * 4 + 3] = t[2];
}
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
struct EpipolarError {
	EpipolarError(cv::Mat p1_, cv::Mat p2_)
		: p1(p1_), p2(p2_) {}

	template <typename T>
	bool operator()(const T * const R_, const T * const t_, T * residual) const
	{
		T rot_matrix[9];
		CalcRodrigues(R_, rot_matrix);

		T t_x[9];
		t_x[0] = T(0.);			t_x[1] = -t_[2];		t_x[2] = t_[1];
		t_x[3] = t_[2];			t_x[4] = T(0.);			t_x[5] = -t_[0];
		t_x[6] = -t_[1];		t_x[7] = t_[0];			t_x[8] = T(0.);

		T t_p1[3], t_p2[3];
		t_p1[0] = T(p1.at<double>(0, 0));			t_p1[1] = T(p1.at<double>(1, 0));		t_p1[2] = T(p1.at<double>(2, 0));
		t_p2[0] = T(p2.at<double>(0, 0));			t_p2[1] = T(p2.at<double>(1, 0));		t_p2[2] = T(p2.at<double>(2, 0));

		T F[9];
		matrix_multiply(t_x, rot_matrix, F, 3, 3, 3, 3);

		T t_p3[3];
		matrix_multiply(t_p2, F, t_p3, 1, 3, 3, 3);

		T rst[1];
		matrix_multiply(t_p3, t_p1, rst, 1, 3, 3, 1);

		residual[0] = T(rst[0]);

		return true;
	}

	static ceres::CostFunction * Create(cv::Mat p1_, cv::Mat p2_) {

		return (new ceres::AutoDiffCostFunction<EpipolarError, 1, 3, 3>
			(new EpipolarError(p1_, p2_)));

	}
	cv::Mat p1;
	cv::Mat p2;
};

void epipolarOptimize(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const cv::Mat& K1, const cv::Mat& K2, cv::Mat& opt_R, cv::Mat& opt_t_)
{
	assert(points1.size() == points2.size());
	cv::Mat rot_vec;
	cv::Rodrigues(opt_R, rot_vec);
	double buf_R[3], buf_t[3];
	buf_t[0] = opt_t_.at<double>(0, 0);		buf_t[1] = opt_t_.at<double>(1, 0);		buf_t[2] = opt_t_.at<double>(2, 0);
	buf_R[0] = rot_vec.at<double>(0, 0);	buf_R[1] = rot_vec.at<double>(1, 0);	buf_R[2] = rot_vec.at<double>(2, 0);

	//-- 验证对极约束
	ceres::Problem epipolarPro;
	for (size_t i = 0; i < points1.size(); i++)
	{
		cv::Point2d pt1 = pixel2cam(points1[i], K1);
		cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1.);
		cv::Point2d pt2 = pixel2cam(points2[i], K2);
		cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1.);

		ceres::CostFunction * costFunc = EpipolarError::Create(y1, y2);
		epipolarPro.AddResidualBlock(costFunc, NULL, buf_R, buf_t);
	}
	ceres::Solver::Options epipolar_options;
	epipolar_options.linear_solver_type = ceres::DENSE_SCHUR;
	epipolar_options.minimizer_progress_to_stdout = true;
	epipolar_options.gradient_tolerance = 0;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(epipolar_options, &epipolarPro, &reprojection_summary);

	opt_t_.at<double>(0, 0) = buf_t[0];		opt_t_.at<double>(1, 0) = buf_t[1];		opt_t_.at<double>(2, 0) = buf_t[2];
	rot_vec.at<double>(0, 0) = buf_R[0];	rot_vec.at<double>(1, 0) = buf_R[1];	rot_vec.at<double>(2, 0) = buf_R[2];
	cv::Rodrigues(rot_vec, opt_R);
}

