#include "cameraPose.h"



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
void matrix_t(const T src[], T des[], int src_row, int src_col, int des_row, int des_col)
{
	for (size_t i = 0; i < src_row; i++)
	{
		for (size_t j = 0; j < src_col; j++)
		{
			des[j * des_col + i] = src[i * src_col + j];
		}
	}
}

struct EpipolarError {
	EpipolarError(cv::Mat _K1, cv::Mat _K2, cv::Point2d _p1, cv::Point2d _p2)
		: K1_M(_K1), K2_M(_K2), p1(_p1), p2(_p2) {}

	template <typename T>
	bool operator()(const T * const R_, const T * const t_, T * residual) const
	{
		T rot_matrix[9];
		CalcRodrigues(R_, rot_matrix);

		T t_tmp[3];

		T length = ceres::sqrt(t_[0] * t_[0] + t_[1] * t_[1] + t_[2] * t_[2]);
		t_tmp[0] = t_[0] / length;
		t_tmp[1] = t_[1] / length;
		t_tmp[2] = t_[2] / length;

		T t_x[9];
		t_x[0] = T(0.);				t_x[1] = -t_tmp[2];			t_x[2] = t_tmp[1];
		t_x[3] = t_tmp[2];			t_x[4] = T(0.);				t_x[5] = -t_tmp[0];
		t_x[6] = -t_tmp[1];			t_x[7] = t_tmp[0];			t_x[8] = T(0.);

		T E[9];
		matrix_multiply(t_x, rot_matrix, E, 3, 3, 3, 3);
		
		T K2[9], K1[9];
		K2[0] = T(K2_M.at<double>(0, 0));			K2[1] = T(K2_M.at<double>(0, 1));			K2[2] = T(K2_M.at<double>(0, 2));
		K2[3] = T(K2_M.at<double>(1, 0));			K2[4] = T(K2_M.at<double>(1, 1));			K2[5] = T(K2_M.at<double>(1, 2));
		K2[6] = T(K2_M.at<double>(2, 0));			K2[7] = T(K2_M.at<double>(2, 1));			K2[8] = T(K2_M.at<double>(2, 2));

		K1[0] = T(K1_M.at<double>(0, 0));			K1[1] = T(K1_M.at<double>(0, 1));			K1[2] = T(K1_M.at<double>(0, 2));
		K1[3] = T(K1_M.at<double>(1, 0));			K1[4] = T(K1_M.at<double>(1, 1));			K1[5] = T(K1_M.at<double>(1, 2));
		K1[6] = T(K1_M.at<double>(2, 0));			K1[7] = T(K1_M.at<double>(2, 1));			K1[8] = T(K1_M.at<double>(2, 2));

		T K2_inv[9], K1_inv[9];
		matrix_inv(K2, K2_inv,3, 3);
		matrix_inv(K1, K1_inv, 3, 3);
		
		T K2_T[9];
		matrix_t(K2_inv,K2_T,3,3,3,3);

		T tmp_1[9],tmp_2[9], F[9];
		matrix_multiply(K2_T, E, tmp_2, 3, 3, 3, 3);
		matrix_multiply(tmp_2, K1_inv, F, 3, 3, 3, 3);

		T x2[3], x1[3];
		x2[0] = T(p2.x);	x2[1] = T(p2.y);	x2[2] = T(1.);
		x1[0] = T(p1.x);	x1[1] = T(p1.y);	x1[2] = T(1.);

		T L1[3], L2[3];
		matrix_multiply(F, x1, L1, 3, 3, 3, 1);
		T F_t[9];
		matrix_t(F, F_t, 3, 3, 3, 3);
		matrix_multiply(F_t, x2, L2, 3, 3, 3, 1);

		T tmp_3[1], tmp_4[1];
		matrix_multiply(x1, L1, tmp_3, 1, 3, 3, 1);
		matrix_multiply(x2, L2, tmp_4, 1, 3, 3, 1);

		T rst[1];
		rst[0] = ceres::abs(tmp_3[0]) + ceres::abs(tmp_4[0]);

		residual[0] = T(rst[0]);

		return true;
	}

	static ceres::CostFunction * Create(cv::Mat _K1, cv::Mat _K2, cv::Point2d _p1, cv::Point2d _p2) {

		return (new ceres::AutoDiffCostFunction<EpipolarError, 1, 3, 3>
			(new EpipolarError(_K1, _K2, _p1, _p2)));

	}
	cv::Mat K1_M;
	cv::Mat K2_M;
	cv::Point2d p1;
	cv::Point2d p2;
};

void epipolarOptimize(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const cv::Mat& K1, const cv::Mat& K2,const cv::Mat& init_R, const cv::Mat& init_t, cv::Mat& opt_R, cv::Mat& opt_t)
{
	assert(points1.size() == points2.size());
	cv::Mat rot_vec;
	cv::Rodrigues(init_R, rot_vec);
	double buf_R[3], buf_t[3];
	buf_t[0] = init_t.at<double>(0, 0);		buf_t[1] = init_t.at<double>(1, 0);		buf_t[2] = init_t.at<double>(2, 0);
	buf_R[0] = rot_vec.at<double>(0, 0);	buf_R[1] = rot_vec.at<double>(1, 0);	buf_R[2] = rot_vec.at<double>(2, 0);

	//-- ��֤�Լ�Լ��
	ceres::Problem epipolarPro;
	for (size_t i = 0; i < points1.size(); i++)
	{
		/*cv::Point2d pt1 = pixel2cam(points1[i], K1);
		cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1.);
		cv::Point2d pt2 = pixel2cam(points2[i], K2);
		cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1.);*/

		ceres::CostFunction * costFunc = EpipolarError::Create(K1, K2, points1[i], points2[i]);
		epipolarPro.AddResidualBlock(costFunc, NULL, buf_R, buf_t);
	}
	ceres::Solver::Options epipolar_options;
	epipolar_options.linear_solver_type = ceres::DENSE_SCHUR;
	epipolar_options.minimizer_progress_to_stdout = true;
	epipolar_options.gradient_tolerance = 0;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(epipolar_options, &epipolarPro, &reprojection_summary);

	opt_t.at<double>(0, 0) = buf_t[0];		opt_t.at<double>(1, 0) = buf_t[1];		opt_t.at<double>(2, 0) = buf_t[2];
	rot_vec.at<double>(0, 0) = buf_R[0];	rot_vec.at<double>(1, 0) = buf_R[1];	rot_vec.at<double>(2, 0) = buf_R[2];
	cv::Rodrigues(rot_vec, opt_R);
}



void find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2, std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
	std::vector<cv::DMatch> matches;

	//-- ��ʼ��
	cv::Mat descriptors_1, descriptors_2;
	// used in OpenCV3 
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
	cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
	// use this if you are in OpenCV2 
	// Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
	// Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	//-- ��һ��:��� Oriented FAST �ǵ�λ��
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);

	//-- �ڶ���:���ݽǵ�λ�ü��� BRIEF ������
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	//-- ������:������ͼ���е�BRIEF�����ӽ���ƥ�䣬ʹ�� Hamming ����
	std::vector<cv::DMatch> match;
	//BFMatcher matcher ( NORM_HAMMING );
	matcher->match(descriptors_1, descriptors_2, match);

	//-- ���Ĳ�:ƥ����ɸѡ
	double min_dist = 10000, max_dist = 0;

	//�ҳ�����ƥ��֮�����С�����������, ���������Ƶĺ�����Ƶ������֮��ľ���
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = match[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//��������֮��ľ��������������С����ʱ,����Ϊƥ������.����ʱ����С�����ǳ�С,����һ������ֵ30��Ϊ����.
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (match[i].distance <= std::max(2 * min_dist, 30.0))
		{
			matches.push_back(match[i]);
		}
	}

	//-- ��ƥ���ת��Ϊvector<cv::Point2f>����ʽ

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


void pose_estimation_2d2d(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,const cv::Mat& K1,const cv::Mat& K2, cv::Mat& R, cv::Mat& t)
{
	//-- �����������
	cv::Mat fundamental_matrix;
	cv::Mat essential_matrix;
	fundamental_matrix = cv::findFundamentalMat(points1, points2, CV_FM_8POINT);

	//-- ���㱾�ʾ���
	essential_matrix = K2.t() * fundamental_matrix * K1;

	cv::Mat R1, R2;
	cv::decomposeEssentialMat(essential_matrix, R1, R2, t);
	std::cout << "R1: " << R1 << std::endl;
	std::cout << "R2: " << R2 << std::endl;
	std::cout << "t: " << t << std::endl;
	R = R2;
	t = -t;
	//-- �ӱ��ʾ����лָ���ת��ƽ����Ϣ.
	cv::Mat _R, _t;
	double focal_length = K1.at<double>(0,0);
	cv::Point2d principal_point(K1.at<double>(0, 2), K1.at<double>(1, 2));
	cv::recoverPose(essential_matrix, points1, points2, _R, _t, focal_length, principal_point);
	std::cout << "_R: " << _R << std::endl;
	std::cout << "_t: " << _t << std::endl;
}


void pose_estimate(const cv::Mat& img_1,const cv::Mat& img_2,const cv::Mat& K1, const cv::Mat& K2, cv::Mat& init_R,cv::Mat& init_t, cv::Mat& opt_R, cv::Mat& opt_t)
{
	std::vector<cv::Point2f> keypoints_1, keypoints_2;
	find_feature_matches(img_1, img_2, keypoints_1, keypoints_2);
	//-- ��������ͼ����˶�
	pose_estimation_2d2d(keypoints_1, keypoints_2, K1, K2, init_R, init_t);
	init_R.copyTo(opt_R); init_t.copyTo(opt_t);
	epipolarOptimize(keypoints_1, keypoints_2, K1, K2, init_R, init_t, opt_R, opt_t);
}

