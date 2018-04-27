
#include <opencv.hpp>
#include <ceres.h>
#include "optimizationFunction.h"

using namespace cv;
using namespace ceres;


typedef struct PairPoint
{
	Point2d imagePoint;
	Point3d worldPoint;

}PairPoint;

void copyPairPoint(const vector<PairPoint> & src, vector<PairPoint> & des);

void randomMakeObjectPoints(vector<PairPoint> & pairPoints, const Rect rect, const int Z, const size_t nums);

void makeImagePoints(vector<PairPoint> & pairPoints, const Mat K, const Mat T, const Mat D, const Size imageSize, const bool isHaveDistortion);

void object2Cam(vector<PairPoint> & pairPoint, const Mat T, const bool isNormalization);

void getPrincipalNearbyPoints(vector<PairPoint> & prinNearPnts, const vector<PairPoint> & pairPoint, const Point2d prinPnt, const double rate);

void estimateFocus(Mat & K, const vector<PairPoint> & prinNearPnts, const Point2d prinPnt);

void estimateDistortion(Mat & D, const vector<PairPoint> & perfectPnts, const vector<PairPoint> & distortedPnts, const Point2d prinPnt);

void matchPair(vector<PairPoint> & perfectPnts, vector<PairPoint> & distortedPnts);

void reprojectionOptimize(const vector<PairPoint> & pairPoints, Mat & K, Mat & T, Mat & D);

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
	ReprojectionError(Point2d observe, Point3d objection)
	: observe_(observe), objection_(objection) {}

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
		T k_p = T(1.0) + D[0] * r2 + D[1] * r2 * r2;// + distortion[4] * r2 * r2 * r2

		T x_distorted, y_distorted;

		x_distorted = x * k_p + T(2.0) * D[2] * x * y + D[3] * (r2 + T(2.0) * x * x);
		y_distorted = y * k_p + T(2.0) * D[3] * x * y + D[2] * (r2 + T(2.0) * y * y);

		T fx = K[0], fy = K[1], cx = K[2], cy = K[3];

		T predicited_x = fx * x_distorted + cx;
		T predicited_y = fy * y_distorted + cy;

		residual[0] = (T)predicited_x - T(observe_.x);
		residual[1] = (T)predicited_y - T(observe_.y);

		return true;
	}

	static ceres::CostFunction * Create(Point2d observe, Point3d objection) {

		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3, 3, 5>
			(new ReprojectionError(observe, objection)));

	}
	Point2d observe_;
	Point3d objection_;
};

int main()
{
	Mat K = (Mat_<double>(3,3) <<
		520, 0, 540,
		0, 520, 480,
		0, 0, 1);
	Mat D = (Mat_<double>(1,5) <<1e-3,-1e-5,-1e-6,-1e-7,0);

	Mat T = (Mat_<double>(3, 4) <<
		8.12486336e-002,	 -7.45385349e-001,	 6.61664069e-001,	 1.42059540e+002,
		9.94988263e-001,	 9.94789228e-002,	-1.01128444e-002,	 -6.71977615e+001,
		- 5.82836643e-002,	 6.59169674e-001,	 7.49732196e-001,	 1.40762988e+003
		);
	cout << "True value:" << endl;
	cout << "K:" << endl << K << endl;
	cout << "T:" << endl << T << endl;
	cout << "D:" << endl << D << endl;

	Point2d principalPnt(540,480);

	vector<PairPoint> pairPoints;
	//在2000mm * 2000mm * 200mm立方体内随机生成空间点
	randomMakeObjectPoints(pairPoints, Rect(0, 0, 2000, 2000), 200, 1000);
	makeImagePoints(pairPoints, K, T, D, Size(1080, 960), true);

	vector<PairPoint> cameraPoints;
	copyPairPoint(pairPoints, cameraPoints);	
	object2Cam(cameraPoints, T, true);

	vector<PairPoint> prinNearPnts;
	getPrincipalNearbyPoints(prinNearPnts, cameraPoints, principalPnt, 0.05);

	Mat estimateK;
	estimateFocus(estimateK, prinNearPnts, principalPnt);
	
	vector<PairPoint> perfectPnts;
	copyPairPoint(pairPoints, perfectPnts);
	makeImagePoints(perfectPnts, estimateK, T, D, Size(1080, 960), false);

	matchPair(perfectPnts, pairPoints);

	Mat estimateD;
	estimateDistortion(estimateD, perfectPnts, pairPoints, principalPnt);

	estimateD = (Mat_<double>(1, 5) <<
		estimateD.at<double>(0, 0), estimateD.at<double>(1, 0),
		estimateD.at<double>(2, 0), estimateD.at<double>(3, 0), 0);

	cout << "Estimate value:" << endl;
	cout << "K:" << endl << estimateK << endl;
	cout << "T:" << endl << T << endl;
	cout << "D:" << endl << estimateD << endl;

	Mat estimateT;
	T.copyTo(estimateT);
	reprojectionOptimize(pairPoints, estimateK, estimateT, estimateD);

	cout << "Optimized estimate value:" << endl;
	cout << "K:" << endl << estimateK << endl;
	cout << "T:" << endl << estimateT << endl;
	cout << "D:" << endl << estimateD << endl;
	return	0;
}













void reprojectionOptimize(const vector<PairPoint> & pairPoints, Mat & K, Mat & T, Mat & D)
{
	vector< Mat_<float> >optimized_camera_pos, optimized_pose_timestamp, optimized_distortion, optimized_intrinsic;

	double dK[4], dR[3], dt[3], dD[5];

	camera_2_array(K, dK);
	transform2RT(T, dR, dt);
	matrix_2_array(D, dD, 1, 5);

	//重投影优化K, camera_pos, pose_timestamp
	ceres::Problem reprojectionPro;

	for (size_t i = 0; i < pairPoints.size(); i++) {
		ceres::CostFunction * costFunc = ReprojectionError::Create(pairPoints[i].imagePoint, pairPoints[i].worldPoint);
		reprojectionPro.AddResidualBlock(costFunc, NULL, dK, dR, dt, dD);

	}
	ceres::Solver::Options reprojection_options;
	reprojection_options.linear_solver_type = ceres::DENSE_SCHUR;
	reprojection_options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(reprojection_options, &reprojectionPro, &reprojection_summary);

	cout << "\n\n" << reprojection_summary.BriefReport() << "\n";

	//double数组转化为矩阵
	array_2_camera(dK, K);
	RT2transform(dR, dt, T);
	array_2_matrix(D, dD, 1, 5);
}

void matchPair(vector<PairPoint> & perfectPnts, vector<PairPoint> & distortedPnts)
{
	size_t i, j;
	i = j = 0;
	while (i < perfectPnts.size() && j < distortedPnts.size()){
		if (std::abs(perfectPnts[i].worldPoint.x - distortedPnts[i].worldPoint.x) < 10e-6 && std::abs(perfectPnts[i].worldPoint.y - distortedPnts[i].worldPoint.y) < 10e-6)
		{
			i++; j++;
		}
		else{
			if (perfectPnts.size()>distortedPnts.size()){
				perfectPnts.erase(perfectPnts.begin() + i);
			}
			else if (distortedPnts.size() > perfectPnts.size()){
				distortedPnts.erase(distortedPnts.begin() + j);
			}
			else{
				i++; j++;
			}
		}
	}
	assert(perfectPnts.size() == distortedPnts.size());
}

void estimateDistortion(Mat & D, const vector<PairPoint> & perfectPnts, const vector<PairPoint> & distortedPnts, const Point2d prinPnt)
{
	assert(perfectPnts.size() == distortedPnts.size());
	Mat_<double> A(2 * distortedPnts.size(), 4);
	Mat_<double> b(2 * distortedPnts.size(), 1);
	for (size_t i = 0; i < distortedPnts.size(); i++)
	{
		double r2 = pow(perfectPnts[i].imagePoint.x - distortedPnts[i].imagePoint.x, 2) + pow(perfectPnts[i].imagePoint.y - distortedPnts[i].imagePoint.y, 2);
		double du = perfectPnts[i].imagePoint.x - prinPnt.x;
		double dv = perfectPnts[i].imagePoint.y - prinPnt.y;

		A.at<double>(i * 2 + 0, 0) = du * r2;
		A.at<double>(i * 2 + 0, 1) = du * r2 * r2;
		A.at<double>(i * 2 + 0, 2) = 3 * du * du + dv * dv;
		A.at<double>(i * 2 + 0, 3) = 2 * du * dv;

		A.at<double>(i * 2 + 1, 0) = dv * r2;
		A.at<double>(i * 2 + 1, 1) = dv * r2 * r2;
		A.at<double>(i * 2 + 1, 2) = 2 * du * dv;
		A.at<double>(i * 2 + 1, 3) = du * du + 3 * dv * dv;

		b.at<double>(i * 2 + 0, 0) = distortedPnts[i].imagePoint.x - perfectPnts[i].imagePoint.x;
		b.at<double>(i * 2 + 1, 0) = distortedPnts[i].imagePoint.y - perfectPnts[i].imagePoint.y;
	}
	//Mat S, U, VT;
	//cv::SVD::compute(A,U,S,VT,SVD::FULL_UV);
	D = (A.t() * A).inv() * A.t() * b;
}

void estimateFocus(Mat & K, const vector<PairPoint> & prinNearPnts, const Point2d prinPnt)
{
	Mat_<double> A(prinNearPnts.size(), 2);
	Mat_<double> b(prinNearPnts.size(), 1);
	for (size_t i = 0; i < prinNearPnts.size(); i++)
	{
		A.at<double>(i, 0) = prinNearPnts[i].worldPoint.x;
		A.at<double>(i, 1) = prinNearPnts[i].worldPoint.y;

		b.at<double>(i, 0) = prinNearPnts[i].imagePoint.x - prinPnt.x + prinNearPnts[i].imagePoint.y - prinPnt.y;
	}
	//Mat S, U, VT;
	//cv::SVD::compute(A,U,S,VT,SVD::FULL_UV);
	Mat x = (A.t() * A).inv() * A.t() * b;
	K = (Mat_<double>(3, 3) <<
		x.at<double>(0, 0), 0, prinPnt.x,
		0, x.at<double>(1, 0), prinPnt.x,
		0, 0, 1);
}

bool cmp(Point2d a, Point2d b){ return a.x < b.x; }
void getPrincipalNearbyPoints(vector<PairPoint> & prinNearPnts, const vector<PairPoint> & pairPoint, const Point2d prinPnt, const double rate)
{
	vector<Point2d> v;
	for (size_t i = 0; i < pairPoint.size(); i++)
	{
		Point2d pt;
		pt.x = std::pow(pairPoint[i].imagePoint.x - prinPnt.x, 2) + std::pow(pairPoint[i].imagePoint.y - prinPnt.y, 2);
		pt.x = std::sqrt(pt.x);
		pt.y = i;
		v.push_back(pt);
	}
	sort(v.begin(), v.end(), cmp);
	const int max_size = v.size() * (rate > 1 ? 1 : rate);
	for (size_t i = 0; i < max_size; i++)
	{
		prinNearPnts.push_back(pairPoint[(int)v[i].y]);
	}
}
void copyPairPoint(const vector<PairPoint> & src, vector<PairPoint> & des)
{
	for (size_t i = 0; i < src.size(); i++)
	{
		PairPoint pt = src[i];
		des.push_back(pt);
	}
}
void object2Cam(vector<PairPoint> & pairPoint, const Mat T, const bool isNormalization)
{
	//投影
	for (size_t i = 0; i < pairPoint.size(); i++)
	{
		Mat W1 = (Mat_<double>(4, 1) << pairPoint[i].worldPoint.x, pairPoint[i].worldPoint.y, pairPoint[i].worldPoint.z, 1.0);

		Mat W2 = T * W1;

		Point3d pts;
		pts.x = W2.at<double>(0, 0);
		pts.y = W2.at<double>(1, 0);
		pts.z = W2.at<double>(2, 0);
		if (isNormalization){
			pts.x /= pts.z;
			pts.y /= pts.z;
			pts.z /= pts.z;
		}
		pairPoint[i].worldPoint = pts;
	}
}

void randomMakeObjectPoints(vector<PairPoint> & pairPoints, const Rect rect, const int Z, const size_t nums)
{
	for (size_t i = 0; i < nums; i++)
	{
		PairPoint pt;
		pt.worldPoint.x = rand() % (rect.width) + rand()*1.0 / RAND_MAX - rect.width / 2.0;
		pt.worldPoint.y = rand() % (rect.height) + rand()*1.0 / RAND_MAX - rect.height / 2.0;
		pt.worldPoint.z = rand() % Z + rand()*1.0 / RAND_MAX - Z / 2.0;
		//以立方体中心为原点，随机生成一系列空间点
		pairPoints.push_back(pt);
	}
}

void makeImagePoints(vector<PairPoint> & pairPoints, const Mat K, const Mat T, const Mat D, const Size imageSize, const bool isHaveDistortion)
{
	//投影
	for (size_t i = 0; i < pairPoints.size();)
	{
		Mat W1 = (Mat_<double>(4, 1) << pairPoints[i].worldPoint.x, pairPoints[i].worldPoint.y, pairPoints[i].worldPoint.z, 1.0);

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
			pairPoints[i].imagePoint=pts;
			i++;
		}
		else
			pairPoints.erase(pairPoints.begin() + i);
	}
}