


#ifndef __OPTIMIZE__CPP
#define __OPTIMIZE__CPP


#include "optimize.h"



/*
*	@ 1)
*/
bool getCornerPoints(std::vector<cv::Point2d> & cornerPts, std::vector<cv::Point3d> & objectPts_InImage, cv::Mat image, const cv::Size boardSize, const cv::Size squareSize)
{
	for (size_t i = 0; i < boardSize.height; i++)
	{
		for (size_t j = 0; j < boardSize.width; j++)
		{
			cv::Point3d pnt;
			pnt.x = j * squareSize.width;
			pnt.y = i * squareSize.height;
			pnt.z = 0;
			objectPts_InImage.push_back(pnt);
		}
	}
	bool found = findChessboardCorners(image, boardSize, cornerPts, cv::CALIB_CB_ADAPTIVE_THRESH);
	if (!found) {
		return false;
	}
	return true;
}



/*
*	@ 2)
*	  算法流程：
*			1）找出临近图像中心的部分点；
*			2）用部分点求取单应；
*			3）根据单应和主点求取焦距f；
*			4）根据单应和内参求取外参；
*			5）根据单应校正图像；
*			6）求取畸变初值；
*			7）根据重投影误差优化所有结果。上述步骤中的1-2步应该明确化一个函数实现
*
*
*/
/*
*	@ 2.1)
*/
void pointsInRect(const std::vector<cv::Point2d> & imagePts_InImage,const std::vector<cv::Point3d> & objectPts_InImage,	const cv::Rect rect, std::vector<cv::Point2d> & imagePts_NearPP, std::vector<cv::Point2d> & objectPts_NearPP)
{
	int rows = rect.height + rect.y;
	int cols = rect.width + rect.x;
	for (size_t i = rect.x; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			imagePts_NearPP.push_back(imagePts_InImage[i * cols + j]);
			objectPts_NearPP.push_back(cv::Point2d(objectPts_InImage[i * cols + j].x, objectPts_InImage[i * cols + j].y));
		}
	}
}
void getHomography(const std::vector<cv::Point2d> & imagePts_InImage, const std::vector<cv::Point3d> & objectPts_InImage, const cv::Rect rect,	cv::Mat & HH)
{
	std::vector<cv::Point2d> imagePts_NearPP, objectPts_NearPP;
	pointsInRect(imagePts_InImage, objectPts_InImage, rect, imagePts_NearPP, objectPts_NearPP);
	// step2:
	HH = cv::findHomography(objectPts_NearPP, imagePts_NearPP); //获取单应矩阵3 x 3
}
/*
*	@ 2.2)
*/
void estimateFocus(const cv::Mat & H, const cv::Point2d prinPnt, cv::Mat & K)
{
	cv::Mat H2 = H.t();
	double h2[3 + 1][3 + 1] = { 0 };
	for (size_t i = 0; i < H2.rows; i++)
	{
		for (size_t j = 0; j < H2.cols; j++)
		{
			h2[i + 1][j + 1] = H2.at<double>(i, j);
		}
	}

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
	
	double b[2 + 1][3 + 1] = { 0 };
	b[1][1] = V[1][1] - V[1][4] * prinPnt.x + V[1][6] * prinPnt.x * prinPnt.x;
	b[1][2] = V[1][3] - V[1][5] * prinPnt.y + V[1][6] * prinPnt.y * prinPnt.y;
	b[1][3] = V[1][6];
	b[2][1] = V[2][1] - V[2][4] * prinPnt.x + V[2][6] * prinPnt.x * prinPnt.x;
	b[2][2] = V[2][3] - V[2][5] * prinPnt.y + V[2][6] * prinPnt.y * prinPnt.y;
	b[2][3] = V[2][6];

	cv::Mat A = (cv::Mat_<double>(2, 2) <<
		V[1][1] - V[1][4] * prinPnt.x + V[1][6] * prinPnt.x * prinPnt.x,
		V[1][3] - V[1][5] * prinPnt.y + V[1][6] * prinPnt.y * prinPnt.y,
		V[2][1] - V[2][4] * prinPnt.x + V[2][6] * prinPnt.x * prinPnt.x,
		V[2][3] - V[2][5] * prinPnt.y + V[2][6] * prinPnt.y * prinPnt.y
		);
	cv::Mat B = (cv::Mat_<double>(2, 1) << -V[1][6], -V[2][6]);

	cv::Mat X = (A.t() * A).inv() * A.t() * B;
	
	double _x1 = X.at<double>(0, 0);
	double _y1 = X.at<double>(1, 0);
	
	K = (cv::Mat_<double>(3, 3) <<
		std::sqrt(1 / _x1), 0, prinPnt.x,
		0, std::sqrt(1 / _y1), prinPnt.y,
		0, 0, 1
	);
}
/*
*	@ 2.3)
*/
void estimateDistortion(const std::vector<cv::Point2d> & perfectPnts, const std::vector<cv::Point2d> & distortedPnts, const cv::Point2d prinPnt, cv::Mat & dist)
{
	assert(perfectPnts.size() == distortedPnts.size());
	cv::Mat_<double> A(2 * distortedPnts.size(), 2);
	cv::Mat_<double> b(2 * distortedPnts.size(), 1);
	for (size_t i = 0; i < distortedPnts.size(); i++)
	{
		double r2 = pow(perfectPnts[i].x, 2) + pow(perfectPnts[i].y, 2);
		double du = perfectPnts[i].x;
		double dv = perfectPnts[i].y;

		A.at<double>(i * 2 + 0, 0) = du * r2;
		A.at<double>(i * 2 + 0, 1) = du * r2 * r2;
		/*A.at<double>(i * 2 + 0, 2) = 2 * du * dv;
		A.at<double>(i * 2 + 0, 3) = 3 * du * du + dv * dv;*/

		A.at<double>(i * 2 + 1, 0) = dv * r2;
		A.at<double>(i * 2 + 1, 1) = dv * r2 * r2;
		/*A.at<double>(i * 2 + 1, 2) = du * du + 3 * dv * dv;
		A.at<double>(i * 2 + 1, 3) = 2 * du * dv;*/

		b.at<double>(i * 2 + 0, 0) = distortedPnts[i].x - perfectPnts[i].x;
		b.at<double>(i * 2 + 1, 0) = distortedPnts[i].y - perfectPnts[i].y;
	}
	dist = (A.t() * A).inv() * A.t() * b;
	dist = (cv::Mat_<double>(1, 5) << dist.at<double>(0, 0), dist.at<double>(1, 0), 0., 0., 0.);
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
struct ReprojectionError {
	ReprojectionError(cv::Point2d observe, cv::Point3d objection, double u0, double v0)
	: observe_(observe), objection_(objection), _u0(u0), _v0(v0) {}

	template <typename T>
	bool operator()(const T * const K, const T * const R, const T * const t, const T * const D, T * residual) const
	{
		T dT[3 * 4];
		RT2TransformTemplate(R, t, dT);

		T W1[4 * 1];
		W1[0] = T(objection_.x);
		W1[1] = T(objection_.y);
		W1[2] = T(objection_.z);
		W1[3] = T(1.0);

		T W2[3 * 1];
		matrix_multiply(dT, W1, W2, 3, 4, 4, 1);

		T x, y;
		x = W2[0] / W2[2];
		y = W2[1] / W2[2];//归一化坐标

		T r2 = x * x + y * y;
		T k_p = T(1.0) + D[0] * r2 + D[1] * r2 * r2;

		T x_distorted, y_distorted;

		x_distorted = x * k_p;
		y_distorted = y * k_p;

		T fx = K[0], fy = K[0], cx = T(_u0), cy = T(_v0);

		T predicited_x = fx * x_distorted + cx;
		T predicited_y = fy * y_distorted + cy;

		residual[0] = (T)predicited_x - T(observe_.x);
		residual[1] = (T)predicited_y - T(observe_.y);

		return true;
	}

	static ceres::CostFunction * Create(cv::Point2d observe, cv::Point3d objection, double u0, double v0) {

		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 1, 3, 3, 5>
			(new ReprojectionError(observe, objection, u0, v0)));

	}
	double _u0;
	double _v0;
	cv::Point2d observe_;
	cv::Point3d objection_;
};
void reprojectionOptimize(const std::vector<cv::Point2d> & imagePts_InImage, const std::vector<cv::Point3d> & objectPts_InImage, cv::Mat & K_opt, cv::Mat & rotVect, cv::Mat & tranVect, cv::Mat & dist_opt, double f_bound[2], double k1_bound[2], double k2_bound[2])
{
	assert(imagePts_InImage.size() == objectPts_InImage.size());
	double dK[1], dR[3], dt[3], dD[5];
	dK[0] = (K_opt.at<double>(0, 0) + K_opt.at<double>(1, 1)) / 2.0;

	dR[0] = rotVect.at<double>(0, 0);
	dR[1] = rotVect.at<double>(1, 0);
	dR[2] = rotVect.at<double>(2, 0);

	dt[0] = tranVect.at<double>(0, 0);
	dt[1] = tranVect.at<double>(1, 0);
	dt[2] = tranVect.at<double>(2, 0);

	dD[0] = dist_opt.at<double>(0, 0);
	dD[1] = dist_opt.at<double>(0, 1);
	dD[2] = dist_opt.at<double>(0, 2);
	dD[3] = dist_opt.at<double>(0, 3);
	dD[4] = dist_opt.at<double>(0, 4);

	ceres::Problem reprojectionPro;
	double u0 = K_opt.at<double>(0, 2);
	double v0 = K_opt.at<double>(1, 2);
	for (size_t i = 0; i < objectPts_InImage.size(); i++) {
		ceres::CostFunction * costFunc = ReprojectionError::Create(imagePts_InImage[i], objectPts_InImage[i], u0, v0);
		reprojectionPro.AddResidualBlock(costFunc, NULL, dK, dR, dt, dD);
	}

	if (std::abs(f_bound[0]) > 10e-6){
		reprojectionPro.SetParameterLowerBound(dK, 0, f_bound[0]);
	}
	if (std::abs(f_bound[1]) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dK, 0, f_bound[1]);
	}
	

	if (std::abs(k1_bound[0]) > 10e-6){
		reprojectionPro.SetParameterLowerBound(dD, 0, k1_bound[0]);
	}
	if (std::abs(k1_bound[1]) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dD, 0, k1_bound[1]);
	}
	

	if (std::abs(k2_bound[0]) > 10e-6){
		reprojectionPro.SetParameterLowerBound(dD, 1, k2_bound[0]);
	}
	if (std::abs(k2_bound[1]) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dD, 1, k2_bound[1]);
	}
	

	ceres::Solver::Options reprojection_options;
	reprojection_options.linear_solver_type = ceres::DENSE_SCHUR;
	reprojection_options.minimizer_progress_to_stdout = true;
	reprojection_options.gradient_tolerance = 0;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(reprojection_options, &reprojectionPro, &reprojection_summary);

	K_opt.at<double>(0, 0) = dK[0];
	K_opt.at<double>(1, 1) = dK[0];
	
	rotVect.at<double>(0, 0) = dR[0];
	rotVect.at<double>(1, 0) = dR[1];
	rotVect.at<double>(2, 0) = dR[2];

	tranVect.at<double>(0, 0) = dt[0];
	tranVect.at<double>(1, 0) = dt[1];
	tranVect.at<double>(2, 0) = dt[2];


	dist_opt.at<double>(0, 0) = dD[0];
	dist_opt.at<double>(0, 1) = dD[1];
	dist_opt.at<double>(0, 2) = dD[2];
	dist_opt.at<double>(0, 3) = dD[3];
	dist_opt.at<double>(0, 4) = dD[4];
}
void decomposeHomographyMat(const cv::Mat & H, const cv::Mat & K, cv::Mat & rotMatrix, cv::Mat & tranVect)
{
	cv::Mat T33 = K.inv() * H;

	cv::Mat r1 = (cv::Mat_<double>(3, 1) << T33.at<double>(0, 0), T33.at<double>(1, 0), T33.at<double>(2, 0));
	cv::Mat r2 = (cv::Mat_<double>(3, 1) << T33.at<double>(0, 1), T33.at<double>(1, 1), T33.at<double>(2, 1));
	cv::Mat r3 = r1.cross(r2);
	cv::Mat m_r1 = r1.t() * r1;
	cv::Mat m_r2 = r2.t() * r2;
	cv::Mat m_r3 = r3.t() * r3;
	double lamda1 = std::sqrt(1.0 / m_r1.at<double>(0, 0));
	double lamda2 = std::sqrt(1.0 / m_r2.at<double>(0, 0));
	double lamda3 = std::sqrt(m_r1.at<double>(0, 0) / m_r3.at<double>(0, 0));
	r3 *= lamda3;

	cv::Mat T34 = (cv::Mat_<double>(3, 4) <<
		T33.at<double>(0, 0), T33.at<double>(0, 1), r3.at<double>(0, 0), T33.at<double>(0, 2),
		T33.at<double>(1, 0), T33.at<double>(1, 1), r3.at<double>(1, 0), T33.at<double>(1, 2),
		T33.at<double>(2, 0), T33.at<double>(2, 1), r3.at<double>(2, 0), T33.at<double>(2, 2)
		);
	T34 *= lamda1;

	rotMatrix = (cv::Mat_<double>(3, 3) <<
		T34.at<double>(0, 0), T34.at<double>(0, 1), T34.at<double>(0, 2),
		T34.at<double>(1, 0), T34.at<double>(1, 1), T34.at<double>(1, 2),
		T34.at<double>(2, 0), T34.at<double>(2, 1), T34.at<double>(2, 2)
		);
	tranVect = (cv::Mat_<double>(3, 1) << T34.at<double>(0, 3), T34.at<double>(1, 3), T34.at<double>(2, 3));
}
void projectPoint(const std::vector<cv::Point2d> & src, const cv::Mat & H33, std::vector<cv::Point2d> & des)
{
	for (size_t i = 0; i < src.size(); i++)
	{
		cv::Mat W1;
		W1 = (cv::Mat_<double>(3, 1) << src[i].x, src[i].y, 1.0);


		cv::Mat W2 = H33 * W1;

		cv::Point3d pts;
		pts.x = W2.at<double>(0, 0);
		pts.y = W2.at<double>(1, 0);
		pts.z = W2.at<double>(2, 0);

		if (std::abs(pts.z) < 10e-6)
			pts.z = 1;

		cv::Point2d pp;
		pp.x = pts.x / pts.z;
		pp.y = pts.y / pts.z;

		des.push_back(pp);
	}
}
void runOptimize(const std::vector<cv::Point2d> & imagePts_InImage, const std::vector<cv::Point3d> & objectPts_InImage,	const cv::Point2d principalPt, cv::Mat & K_init, cv::Mat & rotMatrix_init, cv::Mat & tranVect_init, cv::Mat & dist_init, cv::Mat & K_opt, cv::Mat & rotMatrix_opt, cv::Mat & tranVect_opt, cv::Mat & dist_opt, double f_bound[2], double k1_bound[2], double k2_bound[2], const cv::Rect rect)
{
	cv::Mat HH;
	getHomography(imagePts_InImage, objectPts_InImage, rect, HH); //获取单应矩阵3 x 3

	estimateFocus(HH, principalPt, K_init);  // 输出变量放最后

	decomposeHomographyMat(HH, K_init, rotMatrix_init, tranVect_init);

	cv::Mat rotVect;
	cv::Rodrigues(rotMatrix_init, rotVect);

	std::vector<cv::Point2d> perfectPnts, distortedPnts;
	cv::Mat unitM = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);
	cv::projectPoints(objectPts_InImage, rotVect, tranVect_init, unitM, cv::noArray(), perfectPnts);
	projectPoint(imagePts_InImage, K_init.inv(), distortedPnts);
	if (dist_init.empty()){
		estimateDistortion(perfectPnts, distortedPnts, principalPt, dist_init);
	}

	K_init.copyTo(K_opt);
	tranVect_init.copyTo(tranVect_opt);
	dist_init.copyTo(dist_opt);
	reprojectionOptimize(imagePts_InImage, objectPts_InImage, K_opt, rotVect, tranVect_opt, dist_opt, f_bound, k1_bound, k2_bound);	// 其中f_bound = [f_lowerBound, f_upperBound]; 

	cv::Rodrigues(rotVect, rotMatrix_opt);
}



bool imageRun(const cv::Mat & realImg, const cv::Size boardSize, const cv::Size squareSize, const cv::Point2d principalPt, double & focalLength, cv::Mat & rotMatrix, cv::Mat & tranVect, cv::Mat & dist, double f_bound[2], double k1_bound[2], double k2_bound[2])
{
	// step 1: 提取角点或sift点
	std::vector<cv::Point2d> imagePts_InImage;
	std::vector<cv::Point3d> objectPts_InImage;
	if (getCornerPoints(imagePts_InImage, objectPts_InImage, realImg, boardSize, squareSize)){
		// step 2: runOptimize
		cv::Mat K_init, rotMatrix_init, tranVect_init, dist_init;
		cv::Mat K_opt;
		runOptimize(imagePts_InImage, objectPts_InImage, principalPt, K_init, rotMatrix_init, tranVect_init, dist_init, K_opt, rotMatrix, tranVect, dist, f_bound, k1_bound, k2_bound, cv::Rect(boardSize.width / 4, boardSize.height / 4, boardSize.width / 2, boardSize.height / 2));
		focalLength = K_opt.at<double>(0, 0);

		return true;
	}
	return false;
}



#endif // __OPTIMIZE__CPP
