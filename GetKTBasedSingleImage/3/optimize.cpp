/*
*	@author: liphone
*	@date:	2018/05/08	22:27
*
*/
#include "optimize.h"



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
		T k_p = T(1.0) + D[0] * r2 + D[1] * r2 * r2;// + distortion[4] * r2 * r2 * r2

		T x_distorted, y_distorted;

		x_distorted = x * k_p;// +T(2.0) * D[2] * x * y + D[3] * (r2 + T(2.0) * x * x);
		y_distorted = y * k_p;// +T(2.0) * D[3] * x * y + D[2] * (r2 + T(2.0) * y * y);

		T fx = K[0], fy = K[0], cx = T(_u0), cy = T(_v0);

		T predicited_x = fx * x_distorted + cx;
		T predicited_y = fy * y_distorted + cy;

		residual[0] = (T)predicited_x - T(observe_.x);
		residual[1] = (T)predicited_y - T(observe_.y);

		return true;
	}

	static ceres::CostFunction * Create(cv::Point2d observe, cv::Point3d objection, double u0, double v0) {

		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 2, 3, 3, 5>
			(new ReprojectionError(observe, objection, u0, v0)));

	}
	double _u0;
	double _v0;
	cv::Point2d observe_;
	cv::Point3d objection_;
};



/*
*	0)
*	@function :
*		get a pair of points from the plane points to its image points, out the struct of "PairPoint".
*/
void getPairPoints(std::vector<PairPoint> & pairPoints, const std::vector<cv::Point2d> & imagePnts, const std::vector<cv::Point3d> & worldPnts)
{
	assert(imagePnts.size() == worldPnts.size());
	for (size_t i = 0; i < imagePnts.size(); i++)
	{
		PairPoint pp;
		pp.imagePoint = imagePnts[i];
		pp.worldPoint = worldPnts[i];
		pairPoints.push_back(pp);
	}
}



/*
*	1）
*	@function :
*		accroding to the rate of your setting, get the nearby points of the principal point.
*/
bool cmp(cv::Point2d a, cv::Point2d b){ return a.x < b.x; }
void getPrincipalNearbyPoints(std::vector<PairPoint> & prinNearPnts, const std::vector<PairPoint> & pairPoint, const cv::Point2d prinPnt, const double rate)
{
	std::vector<cv::Point2d> v;
	for (size_t i = 0; i < pairPoint.size(); i++)
	{
		cv::Point2d pt;
		pt.x = std::pow(pairPoint[i].imagePoint.x - prinPnt.x, 2) + std::pow(pairPoint[i].imagePoint.y - prinPnt.y, 2);
		pt.x = std::sqrt(pt.x);
		pt.y = i;
		v.push_back(pt);
	}
	sort(v.begin(), v.end(), cmp);
	int max_size = 0;
	if (rate > 4)
	{
		max_size = rate;
	}
	else if (rate > v.size()){
		max_size = v.size();
	}
	else if (rate > 0 && rate <= 1){
		max_size = rate * v.size();
	}
	for (size_t i = 0; i < max_size; i++)
	{
		prinNearPnts.push_back(pairPoint[(int)v[i].y]);
	}
}



/*
*	2)
*	@function :
*		accroding to the transform matrix H from a plane points to its image points 
*	and the principal points in the camera intrinsic parameters, get the camera's focus.
*/
void estimateFocus(cv::Mat & K, const cv::Mat & H, const cv::Point2d prinPnt)
{
	/*double h[3 + 1][3 + 1] = { 0 };
	for (size_t i = 0; i < H.rows; i++)
	{
		for (size_t j = 0; j < H.cols; j++)
		{
			h[i + 1][j + 1] = H.at<double>(i, j);
		}
	}
	double _a[6] = { 0 }, _b[6] = { 0 };
	_a[1] = h[1][1] * h[3][2] + h[1][2] * h[3][1];
	_a[2] = h[2][2] * h[3][1] + h[2][1] * h[3][2];
	_a[3] = h[1][1] * h[1][2];
	_a[4] = h[2][1] * h[2][2];
	_a[5] = h[3][1] * h[3][2];
	_b[1] = 2 * (h[1][1] * h[3][1] - h[1][2] * h[3][2]);
	_b[2] = 2 * (h[2][1] * h[3][1] - h[2][2] * h[3][2]);
	_b[3] = h[1][1] * h[1][1] - h[1][2] * h[1][2];
	_b[4] = h[2][1] * h[2][1] - h[2][2] * h[2][2];
	_b[5] = h[3][1] * h[3][1] - h[3][2] * h[3][2];

	double _c[3][4] = { 0 };
	_c[1][1] = _a[5] * prinPnt.x*prinPnt.x + _a[3] - _a[1] * prinPnt.x;
	_c[1][2] = _a[5];
	_c[1][3] = _a[5] * prinPnt.y*prinPnt.y - _a[2] * prinPnt.y + _a[4];

	_c[2][1] = _b[5] * prinPnt.x*prinPnt.x + _b[3] - _b[1] * prinPnt.x;
	_c[2][2] = _b[5];
	_c[2][3] = _b[5] * prinPnt.y*prinPnt.y - _b[2] * prinPnt.y + _b[4];

	double _x = (_c[1][2] * _c[2][3] - _c[1][3] * _c[2][2]) / (_c[2][2] * _c[1][1] - _c[2][1] * _c[1][2]);
	double _y = (_c[1][1] * _c[2][3] - _c[1][3] * _c[2][1]) / (_c[1][2] * _c[2][1] - _c[1][1] * _c[2][2]);

	double _t = std::sqrt(_x);
	double _f = std::sqrt(_y) / _t;*/

	/*K = (cv::Mat_<double>(3, 3) <<
		_f, 0, prinPnt.x,
		0, _t * _f, prinPnt.y,
		0, 0, 1
		);*/
	
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
	
	double _x1 = X.at<double>(0, 0);// (b[1][2] * b[2][3] - b[1][3] * b[2][2]) / (b[2][2] * b[1][1] - b[2][1] * b[1][2]);
	double _y1 = X.at<double>(1, 0);// (b[1][1] * b[2][3] - b[1][3] * b[2][1]) / (b[1][2] * b[2][1] - b[1][1] * b[2][2]);
	
	K = (cv::Mat_<double>(3, 3) <<
		std::sqrt(1 / _x1), 0, prinPnt.x,
		0, std::sqrt(1 / _y1), prinPnt.y,
		0, 0, 1
	);




}



/*
*	3)
*	@function :
*		copy source pair points to destination pair points.
*/
void copyPairPoint(const std::vector<PairPoint> & src, std::vector<PairPoint> & des)
{
	for (size_t i = 0; i < src.size(); i++)
	{
		PairPoint pt = src[i];
		des.push_back(pt);
	}
}



/*
*	4)
*	@function :
*		using the homography(3 x 3) matrix to make perfect image points.
*/
void makeImagePoints(std::vector<PairPoint> & pairPoints, const cv::Mat H33, const bool isUseImagePoint)
{
	//投影
	for (size_t i = 0; i < pairPoints.size();i++)
	{
		cv::Mat W1;
		if(isUseImagePoint)
			W1 = (cv::Mat_<double>(3, 1) << pairPoints[i].imagePoint.x, pairPoints[i].imagePoint.y, 1.0);
		else
			W1 = (cv::Mat_<double>(3, 1) << pairPoints[i].worldPoint.x, pairPoints[i].worldPoint.y, 1.0);


		cv::Mat W2 = H33 * W1;

		cv::Point3d pts;
		pts.x = W2.at<double>(0, 0);
		pts.y = W2.at<double>(1, 0);
		pts.z = W2.at<double>(2, 0);

		if (std::abs(pts.z) < 10e-6)
			pts.z = 1;

		cv::Point3d pp;
		pp.x = pts.x / pts.z;
		pp.y = pts.y / pts.z;
		pp.z = pts.z / pts.z;

		pairPoints[i].worldPoint = pp;
	}
}



/*
*	5)
*	@status :	deleted.
*	@function :
*		matching the perfect points and distorted points in case of the perfect points or distorted points 
*	is out of the image, in that case erese it.
*/
void matchPair(std::vector<PairPoint> & perfectPnts, std::vector<PairPoint> & distortedPnts)
{
	size_t i, j;
	i = j = 0;
	while (i < perfectPnts.size() && j < distortedPnts.size()){
		if (std::abs(perfectPnts[i].worldPoint.x - distortedPnts[i].worldPoint.x) < 10e-6 && std::abs(perfectPnts[i].worldPoint.y - distortedPnts[i].worldPoint.y) < 10e-6)
		{
			i++; j++;
		}
		else{
			if (perfectPnts.size() > distortedPnts.size()){
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



/*
*	6)
*	@function :
*		using the perfect points and distorted point to estimate distortion.
*/
void estimateDistortion(cv::Mat & D, const std::vector<PairPoint> & perfectPnts, const std::vector<PairPoint> & distortedPnts, const cv::Point2d prinPnt)
{
	assert(perfectPnts.size() == distortedPnts.size());
	cv::Mat_<double> A(2 * distortedPnts.size(), 2);
	cv::Mat_<double> b(2 * distortedPnts.size(), 1);
	for (size_t i = 0; i < distortedPnts.size(); i++)
	{
		double r2 = pow(perfectPnts[i].worldPoint.x, 2) + pow(perfectPnts[i].worldPoint.y, 2);
		double du = perfectPnts[i].worldPoint.x;
		double dv = perfectPnts[i].worldPoint.y;

		A.at<double>(i * 2 + 0, 0) = du * r2;
		A.at<double>(i * 2 + 0, 1) = du * r2 * r2;
		/*A.at<double>(i * 2 + 0, 2) = 2 * du * dv;
		A.at<double>(i * 2 + 0, 3) = 3 * du * du + dv * dv;*/

		A.at<double>(i * 2 + 1, 0) = dv * r2;
		A.at<double>(i * 2 + 1, 1) = dv * r2 * r2;
		/*A.at<double>(i * 2 + 1, 2) = du * du + 3 * dv * dv;
		A.at<double>(i * 2 + 1, 3) = 2 * du * dv;*/

		b.at<double>(i * 2 + 0, 0) = distortedPnts[i].worldPoint.x - perfectPnts[i].worldPoint.x;
		b.at<double>(i * 2 + 1, 0) = distortedPnts[i].worldPoint.y - perfectPnts[i].worldPoint.y;
	}
	D = (A.t() * A).inv() * A.t() * b;
}



/*
*	7)
*	@function :
*		using the ceres to optimize the intrinsic parameter, the extrinsic parameter and the distortion parameter.
*/
void transform2RT(const cv::Mat & T, double R[], double t[])
{

	cv::Mat m_T = T(cv::Rect(0, 0, 3, 3));
	cv::Mat m_R = cv::Mat(3, 1, CV_64FC1);
	Rodrigues(m_T, m_R);
	R[0] = m_R.at<double>(0, 0);
	R[1] = m_R.at<double>(1, 0);
	R[2] = m_R.at<double>(2, 0);


	cv::Mat m_t = cv::Mat(3, 1, CV_64FC1);
	T(cv::Rect(3, 0, 1, 3)).copyTo(m_t.rowRange(0, 3));
	t[0] = m_t.at<double>(0, 0);
	t[1] = m_t.at<double>(1, 0);
	t[2] = m_t.at<double>(2, 0);
}
void RT2transform(const double R[], const double t[], cv::Mat & T)
{
	cv::Mat rvecs = (cv::Mat_<double>(3, 1) << R[0], R[1], R[2]); // 旋转向量
	cv::Mat mR;          // 图像的旋转矩阵 ;

	// 将旋转向量转换为相对应的旋转矩阵 */   
	cv::Rodrigues(rvecs, mR);

	T = (cv::Mat_<double>(4, 4) <<
		mR.at<double>(0, 0), mR.at<double>(0, 1), mR.at<double>(0, 2), t[0],
		mR.at<double>(1, 0), mR.at<double>(1, 1), mR.at<double>(1, 2), t[1],
		mR.at<double>(2, 0), mR.at<double>(2, 1), mR.at<double>(2, 2), t[2],
		0, 0, 0, 1);
}
void camera_2_array(const cv::Mat & K, double arr[])
{
	arr[0] = K.at<double>(0, 0);
	arr[1] = K.at<double>(1, 1);
	arr[2] = K.at<double>(0, 2);
	arr[3] = K.at<double>(1, 2);
}
void array_2_camera(const double arr[], cv::Mat & K)
{
	float arr2[] = { arr[0], arr[1], arr[2], arr[3] };
	K = (cv::Mat_<double>(3, 3) <<
		arr2[0], 0, arr2[2],
		0, arr2[1], arr2[3],
		0, 0, 1);
}
bool matrix_2_array(const cv::Mat mat, double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < arr_row; i++) {
		for (int j = 0; j < arr_col; j++) {
			if (i * arr_col + j < arr_row * arr_col) {
				if (i < mat.rows && j < mat.cols)
					arr[i * arr_col + j] = mat.at<double>(i, j);
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
bool array_2_matrix(cv::Mat & mat, const double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			if (i < mat.rows && j < mat.cols) {
				if (i * arr_col + j < arr_row * arr_col)
					mat.at<double>(i, j) = arr[i * arr_col + j];
				else if (i == j)
					mat.at<double>(i, j) = 1.0;
				else
					mat.at<double>(i, j) = 0.0;
			}
			//cout << mat.at<double>(i, j) << "\t";
		}
		//cout << "\n";
	}
	return true;
}

void reprojectionOptimize(const std::vector<PairPoint> & pairPoints, cv::Mat & K, cv::Mat & T, cv::Mat & D, const double f_section,const double k0_section,const double k1_section)
{
	double dK[2], dR[3], dt[3], dD[5];
	dK[0] = K.at<double>(0, 0);
	dK[1] = K.at<double>(1, 1);
	transform2RT(T, dR, dt);
	matrix_2_array(D, dD, 1, 5);

	//重投影优化K, camera_pos, pose_timestamp
	ceres::Problem reprojectionPro;
	double u0 = K.at<double>(0, 2);
	double v0 = K.at<double>(1, 2);
	for (size_t i = 0; i < pairPoints.size(); i++) {
		ceres::CostFunction * costFunc = ReprojectionError::Create(pairPoints[i].imagePoint, pairPoints[i].worldPoint,u0,v0);
		reprojectionPro.AddResidualBlock(costFunc, NULL, dK, dR, dt, dD);
	}
	double const_f0 = dK[0];
	double const_k0 = dD[0];
	double const_k1 = dD[1];
	if (std::abs(f_section) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dK, 0, const_f0 + f_section);
		reprojectionPro.SetParameterLowerBound(dK, 0, const_f0 - f_section);
	}

	if (std::abs(k0_section) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dD, 0, const_k0 + k0_section);
		reprojectionPro.SetParameterLowerBound(dD, 0, const_k0 - k0_section);
	}

	if (std::abs(k1_section) > 10e-6){
		reprojectionPro.SetParameterUpperBound(dD, 1, const_k1 + k1_section);
		reprojectionPro.SetParameterLowerBound(dD, 1, const_k1 - k1_section);
	}

	ceres::Solver::Options reprojection_options;
	reprojection_options.linear_solver_type = ceres::DENSE_SCHUR;
	reprojection_options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary reprojection_summary;
	ceres::Solve(reprojection_options, &reprojectionPro, &reprojection_summary);

	//cout << "\n\n" << reprojection_summary.BriefReport() << "\n";

	//double数组转化为矩阵
	K.at<double>(0, 0) = dK[0];
	K.at<double>(1, 1) = dK[0];

	RT2transform(dR, dt, T);
	array_2_matrix(D, dD, 1, 5);
}



/*
*	8)
*	@function :
*		an encapsulation of the process produce of this file.
*/
void object2Cam(std::vector<PairPoint> & pairPoint, const cv::Mat T, const bool isNormalization)
{
	//投影
	for (size_t i = 0; i < pairPoint.size(); i++)
	{
		cv::Mat W1 = (cv::Mat_<double>(4, 1) << pairPoint[i].worldPoint.x, pairPoint[i].worldPoint.y, pairPoint[i].worldPoint.z, 1.0);

		cv::Mat W2 = T * W1;

		cv::Point3d pts;
		pts.x = W2.at<double>(0, 0);
		pts.y = W2.at<double>(1, 0);
		pts.z = W2.at<double>(2, 0);
		if (isNormalization){
			if (std::abs(pts.z) < 10e-6)
				pts.z = 1;
			cv::Point3d pp;
			pp.x = pts.x / pts.z;
			pp.y = pts.y / pts.z;
			pp.z = pts.z / pts.z;
			pairPoint[i].worldPoint = pp;
		}
		else{
			pairPoint[i].worldPoint = pts;
		}
	}
}
void estimateOptimize(std::vector<PairPoint> & pairPoints, const cv::Point2d principalPnt, const cv::Size imageSize, cv::Mat & optiK, cv::Mat & optiT, cv::Mat & optiD, cv::Mat & estimateK, cv::Mat & estimateT, cv::Mat & estimateD, const double rate, const double f_section, const double k0_section, const double k1_section)
{
	std::vector<PairPoint> prinNearPnts;
	getPrincipalNearbyPoints(prinNearPnts, pairPoints, principalPnt, rate); // 获取主点附近点

	cv::Mat_<double> A(prinNearPnts.size(), 2), B(prinNearPnts.size(), 2);
	for (size_t i = 0; i < prinNearPnts.size(); i++)
	{
		A.at<double>(i, 0) = prinNearPnts[i].worldPoint.x;
		A.at<double>(i, 1) = prinNearPnts[i].worldPoint.y;
		B.at<double>(i, 0) = prinNearPnts[i].imagePoint.x;
		B.at<double>(i, 1) = prinNearPnts[i].imagePoint.y;
	}
	cv::Mat estimateH = cv::findHomography(A, B); //获取单应矩阵3 x 3

	/*std::vector<PairPoint>vtest;
	copyPairPoint(pairPoints, vtest);
	makeImagePoints(vtest, estimateH, false);*/

	
	estimateFocus(estimateK, estimateH, principalPnt); // 获取焦距f和系数t

	cv::Mat T33 = estimateK.inv() * estimateH;

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

	estimateT = (cv::Mat_<double>(3, 4) <<
		T33.at<double>(0, 0), T33.at<double>(0, 1), r3.at<double>(0, 0), T33.at<double>(0, 2),
		T33.at<double>(1, 0), T33.at<double>(1, 1), r3.at<double>(1, 0), T33.at<double>(1, 2),
		T33.at<double>(2, 0), T33.at<double>(2, 1), r3.at<double>(2, 0), T33.at<double>(2, 2)
		);
	estimateT *= lamda1;
	

	std::vector<PairPoint> perfectPnts;
	copyPairPoint(pairPoints, perfectPnts); //copy
	object2Cam(perfectPnts, estimateT, true);
	std::vector<PairPoint> distortedPnts;
	copyPairPoint(pairPoints, distortedPnts); //copy
	makeImagePoints(distortedPnts, estimateK.inv(), true); //获取畸变归一化图像点

	/*std::vector<PairPoint>vd, vp;
	copyPairPoint(distortedPnts, vd);
	copyPairPoint(perfectPnts, vp);
	makeImagePoints(vp, estimateK, false);
	makeImagePoints(vd, estimateK, false);*/

	estimateDistortion(estimateD, perfectPnts, distortedPnts, principalPnt); //获取畸变参数

	estimateD = (cv::Mat_<double>(1, 5) <<
		estimateD.at<double>(0, 0), estimateD.at<double>(1, 0),
		0, 0, 0);


	estimateK.copyTo(optiK);
	estimateT.copyTo(optiT);
	estimateD.copyTo(optiD);
	reprojectionOptimize(pairPoints, optiK, optiT, optiD, f_section, k0_section, k1_section);
}



/*
*	9)
*	@function :
*		a construction of the function estimateOptimize.
*/
void estimateOptimize(const std::vector<cv::Point2d> & imagePnts, const std::vector<cv::Point3d> & worldPnts, const cv::Point2d principalPnt, const cv::Size imageSize, cv::Mat & optiK, cv::Mat & optiT, cv::Mat & optiD, cv::Mat & estimateK, cv::Mat & estimateT, cv::Mat & estimateD, const double rate, const double f_section, const double k0_section, const double k1_section)
{
	std::vector<PairPoint> pairPoints;
	getPairPoints(pairPoints, imagePnts, worldPnts);
	estimateOptimize(pairPoints, principalPnt, imageSize, optiK, optiT, optiD, estimateK, estimateT, estimateD, rate);
}




