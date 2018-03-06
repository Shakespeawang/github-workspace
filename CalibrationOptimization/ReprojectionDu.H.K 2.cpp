//#include <cmath>
//#include <cstdio>
//#include <iostream>
//
//#include <iterator>
//
//#include "ceres/ceres.h"
//#include "ceres/rotation.h"
//
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv/cxcore.h>
//#include <vector>
//
//using namespace cv;
//using namespace std;
//
//template<typename T>
//void ChengRR(const T a[9], const T b[9], T c[9])
//{
//
//	c[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
//	c[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
//	c[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
//	c[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
//	c[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
//	c[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
//	c[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
//	c[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
//	c[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
//
//
//}
//
//template<typename T>
//void ChengRT(const T a[9], const T b[3], T c[3])
//{
//
//	c[0] = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
//	c[1] = a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
//	c[2] = a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
//
//
//}
//
//template<typename T>
//void JiaTT(const T a[3], const T b[3], T c[3])
//{
//
//	c[0] = a[0] + b[0];
//	c[1] = a[1] + b[1];
//	c[2] = a[2] + b[2];
//
//}
//
//template<typename T>
//void ChengRTX(const T a[9], const T b[3], const T c[3], T d[3])
//{
//
//	d[0] = a[0] * c[0] + a[1] * c[1] + a[2] * c[2] + b[0];
//	d[1] = a[3] * c[0] + a[4] * c[1] + a[5] * c[2] + b[1];
//	d[2] = a[6] * c[0] + a[7] * c[1] + a[8] * c[2] + b[2];
//
//}
//
//template<typename T>
//void CalcRodrigues(const T a[3], T b[9])
//{
//	if (a[0] == (T)0.0&&a[1] == (T)0.0&&a[2] == (T)0.0)
//	{
//		b[0] = (T)1.0;
//		b[1] = (T)0.0;
//		b[2] = (T)0.0;
//		b[3] = (T)0.0;
//		b[4] = (T)1.0;
//		b[5] = (T)0.0;
//		b[6] = (T)0.0;
//		b[7] = (T)0.0;
//		b[8] = (T)1.0;
//	}
//	else
//	{
//		T angle = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
//		T u[3] = { a[0] / angle, a[1] / angle, a[2] / angle };
//		b[0] = cos(angle) + u[0] * u[0] * (1.0 - cos(angle));
//		b[1] = u[0] * u[1] * (1.0 - cos(angle)) - u[2] * sin(angle);
//		b[2] = u[1] * sin(angle) + u[0] * u[2] * (1.0 - cos(angle));
//
//		b[3] = u[2] * sin(angle) + u[0] * u[1] * (1.0 - cos(angle));
//		b[4] = cos(angle) + u[1] * u[1] * (1.0 - cos(angle));
//		b[5] = -u[0] * sin(angle) + u[1] * u[2] * (1.0 - cos(angle));
//
//		b[6] = -u[1] * sin(angle) + u[0] * u[2] * (1.0 - cos(angle));
//		b[7] = u[0] * sin(angle) + u[1] * u[2] * (1.0 - cos(angle));
//		b[8] = cos(angle) + u[2] * u[2] * (1.0 - cos(angle));
//	}
//
//}
//
//static void calcChessboardCorners(Size boardSize, double squareSize, vector<Point3d>& corners)
//{
//	corners.resize(0);
//	for (int i = 0; i < boardSize.height; i++)        //height和width位置不能颠倒
//	for (int j = 0; j < boardSize.width; j++)
//	{
//		corners.push_back(Point3d(j*squareSize, i*squareSize, 0));
//	}
//}
//struct SnavelyReprojectionError {
//	SnavelyReprojectionError(cv::Point2d& observed, cv::Point3d objection, int a, int b, int c)
//	: observed(observed), objection(objection), a(a), b(b), c(c){}
//
//	template <typename T>
//	bool operator()(const T* const in, const T* const r1, const T* const t1, const T* const r2, const T* const t2, const T* const dis, T* residuals) const
//	{
//		T om1[9], om2[9];
//		T rx[9], tx1[3], tx[3];
//
//		CalcRodrigues(r1, om1);
//		CalcRodrigues(r2, om2);
//
//
//		ChengRR(om1, om2, rx);
//		ChengRT(om1, t2, tx1);
//		JiaTT(t1, tx1, tx);
//
//		T fx = in[0];
//		T fy = in[1];
//		T cx = in[2];
//		T cy = in[3];
//
//		T pos3d[3], p[3];
//		pos3d[0] = (T)objection.x;
//		pos3d[1] = (T)objection.y;
//		pos3d[2] = (T)objection.z;
//		ChengRTX(rx, tx, pos3d, p);
//
//		T x = p[0] / p[2];
//		T y = p[1] / p[2];
//		T r22 = x*x + y*y;
//		T distortion = T(1.0) + r22  * (dis[0] + dis[1] * r22);
//
//		int i = a;
//		int j = b;
//		int k = c;
//
//		T predicited1 = fx*x*distortion + cx;
//		T predicited2 = fy*y*distortion + cy;
//
//		residuals[0] = predicited1 - T(observed.x);
//		residuals[1] = predicited2 - T(observed.y);
//
//		//cout<<residuals[0]<<" "<<residuals[1]<<endl;
//
//		return true;
//	}
//
//	// Factory to hide the construction of the CostFunction object from
//	// the client code.
//	static ceres::CostFunction* Create(cv::Point2d& observed, cv::Point3d objection, const int a, const int b, const int c) {
//		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 4, 3, 3, 3, 3, 2>(
//			new SnavelyReprojectionError(observed, objection, a, b, c)));
//	}
//
//	double k1;
//	double k2;
//	int a;
//	int b;
//	int c;
//	cv::Point2d observed;
//	cv::Point3d objection;
//	//double observed_x;
//	//double observed_y;
//	//double d1;
//	//double d2;
//	//double d3;
//	//double a1;
//	//double a2;
//	//double a3;
//	//double in1;
//	//double in2;
//	//double in3;
//	//double in4;
//
//	//double k3;
//	//double k4;
//
//};
//
//struct SnavelyReprojectionError2 {
//	SnavelyReprojectionError2(cv::Point2d& observed, cv::Point3d objection, int a, int b, int c)
//	: observed(observed), objection(objection), a(a), b(b), c(c){}
//
//	template <typename T>
//	bool operator()(const T* const in, const T* const r2, const T* const t2, const T* const dis, T* residuals) const
//	{
//
//		T rx[9], tx[3];
//
//		CalcRodrigues(r2, rx);
//		tx[0] = t2[0];
//		tx[1] = t2[1];
//		tx[2] = t2[2];
//
//
//
//		T fx = in[0];
//		T fy = in[1];
//		T cx = in[2];
//		T cy = in[3];
//
//		T pos3d[3], p[3];
//		pos3d[0] = (T)objection.x;
//		pos3d[1] = (T)objection.y;
//		pos3d[2] = (T)objection.z;
//		ChengRTX(rx, tx, pos3d, p);
//
//		T x = p[0] / p[2];
//		T y = p[1] / p[2];
//		T r22 = x*x + y*y;
//		T distortion = T(1.0) + r22  * (dis[0] + dis[1] * r22);
//
//		int i = a;
//		int j = b;
//		int k = c;
//
//		T predicited1 = fx*x*distortion + cx;
//		T predicited2 = fy*y*distortion + cy;
//
//		residuals[0] = predicited1 - T(observed.x);
//		residuals[1] = predicited2 - T(observed.y);
//
//		//cout<<residuals[0]<<" "<<residuals[1]<<endl;
//
//		return true;
//	}
//
//	// Factory to hide the construction of the CostFunction object from
//	// the client code.
//	static ceres::CostFunction* Create(cv::Point2d& observed, cv::Point3d objection, const int a, const int b, const int c) {
//		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError2, 2, 4, 3, 3, 2>(
//			new SnavelyReprojectionError2(observed, objection, a, b, c)));
//	}
//
//	double k1;
//	double k2;
//	int a;
//	int b;
//	int c;
//	cv::Point2d observed;
//	cv::Point3d objection;
//	//double observed_x;
//	//double observed_y;
//	//double d1;
//	//double d2;
//	//double d3;
//	//double a1;
//	//double a2;
//	//double a3;
//	//double in1;
//	//double in2;
//	//double in3;
//	//double in4;
//
//	//double k3;
//	//double k4;
//
//};
//
//int main()
//{
//	double dis[11][2] = { 0.0 };
//	string filename1 = "D:\\毕业设计\\图漾第二次标定数据\\cameras_intrinsic_parameters1.xml";
//	string filename2 = "D:\\毕业设计\\图漾第二次标定数据\\multi-extrinsic-results1.xml";
//	FileStorage intrinsic(filename1, FileStorage::READ);
//	FileStorage extrinsic(filename2, FileStorage::READ);
//	vector<Mat> intrin;
//	vector<Mat> R1;
//	vector<Mat> T1;
//	vector<Mat> R2;
//	vector<Mat> T2;
//	double INN[11][4];
//	double RR1[11][3];
//	double TT1[11][3];
//	double RR2[11][3];
//	double TT2[11][3];
//	for (int i = 0; i<11; i++)
//	{
//		stringstream kmat, r0mat, rmat;
//		kmat << "K_" << i;
//		r0mat << "camera_pose_" << i;
//		rmat << "pose_timestamp_" << i;
//		Mat in(3, 3, CV_64FC1);
//		Mat ex0(4, 4, CV_64FC1);
//		Mat ex(4, 4, CV_64FC1);
//		intrinsic[kmat.str()] >> in;
//		//cout<<in<<endl;
//		double a[4] = { 0 };
//		//if(i<7)
//		//{
//		INN[i][0] = in.at<float>(0, 0);
//		INN[i][1] = in.at<float>(1, 1);
//		INN[i][2] = in.at<float>(0, 2);
//		INN[i][3] = in.at<float>(1, 2);
//		//}
//		/*else
//		{
//		INN[i][0]=in.at<float>(0,0)*0.5;
//		INN[i][1]=in.at<float>(1,1)*0.5;
//		INN[i][2]=in.at<float>(0,2)*0.5;
//		INN[i][3]=in.at<float>(1,2)*0.5;
//		}*/
//		Mat intr = Mat(4, 1, CV_64FC1, RR1);
//		//cout<<intr<<endl;
//		intrin.push_back(intr);
//
//		extrinsic[r0mat.str()] >> ex0;
//		//cout<<ex0<<endl;
//		//Mat ex3=ex0.inv();
//		//cout<<ex3<<endl;
//		Mat om1 = ex0(Rect(0, 0, 3, 3));
//		Mat omi1 = Mat(3, 1, CV_64FC1);
//		Rodrigues(om1, omi1);
//		Mat t1 = Mat(3, 1, CV_64FC1);
//		ex0(Rect(3, 0, 1, 3)).copyTo(t1.rowRange(0, 3));
//		R1.push_back(omi1);
//		T1.push_back(t1);
//		RR1[i][0] = omi1.at<float>(0, 0);
//		RR1[i][1] = omi1.at<float>(1, 0);
//		RR1[i][2] = omi1.at<float>(2, 0);
//		TT1[i][0] = t1.at<double>(0, 0);
//		TT1[i][1] = t1.at<double>(1, 0);
//		TT1[i][2] = t1.at<double>(2, 0);
//		/*cout<<omi1<<endl;
//		cout<<t1<<endl;*/
//
//		extrinsic[rmat.str()] >> ex;
//		//Mat ex2=ex.inv();
//		Mat om2 = ex(Rect(0, 0, 3, 3));
//		Mat omi2 = Mat(3, 1, CV_64FC1);
//		Rodrigues(om2, omi2);
//		Mat t2 = Mat(3, 1, CV_64FC1);
//		ex(Rect(3, 0, 1, 3)).copyTo(t2.rowRange(0, 3));
//		R2.push_back(omi2);
//		T2.push_back(t2);
//		RR2[i][0] = omi2.at<float>(0, 0);
//		RR2[i][1] = omi2.at<float>(1, 0);
//		RR2[i][2] = omi2.at<float>(2, 0);
//		TT2[i][0] = t2.at<double>(0, 0);
//		TT2[i][1] = t2.at<double>(1, 0);
//		TT2[i][2] = t2.at<double>(2, 0);
//		//cout<<omi2<<endl<<t2<<endl;
//
//
//	}
//	/*cout<<intrin.size()<<endl;
//	cout<<R1.size()<<endl;
//	cout<<T1.size()<<endl;
//	cout<<R2.size()<<endl;
//	cout<<T2.size()<<endl;*/
//	ceres::Problem problem;
//	/*for(int i=0;i<1;i++)
//	{
//	problem.AddParameterBlock(intrin[i].ptr<double>(), 4);
//	problem.AddParameterBlock(R1[i].ptr<double>(), 3);
//	problem.AddParameterBlock(T1[i].ptr<double>(), 3);
//	problem.AddParameterBlock(R2[i].ptr<double>(), 3);
//	problem.AddParameterBlock(T2[i].ptr<double>(), 3);
//	}*/
//
//
//	for (int i = 0; i<11; i++)
//	{
//		cout << "读取数据" << i << endl;
//		/*problem.AddParameterBlock(intrin[i].ptr<double>(), 4);
//		problem.AddParameterBlock(R1[i].ptr<double>(), 3);
//		problem.AddParameterBlock(T1[i].ptr<double>(), 3);*/
//
//		for (int j = 0; j<11; j++)
//		{
//			/*problem.AddParameterBlock(R2[j].ptr<double>(), 3);
//			problem.AddParameterBlock(T2[j].ptr<double>(), 3);*/
//			stringstream filename;
//			filename << "D:\\毕业设计\\图漾第二次标定数据\\0-" << i << "-" << j << ".png";
//			Mat image = imread(filename.str(), 0);
//			Size board_size = Size(9, 6);
//			vector<Point2d> obser;
//			bool found1 = findChessboardCorners(image, board_size, obser, CALIB_CB_ADAPTIVE_THRESH);
//
//			cout << "读取棋盘格" << j << " " << found1 << endl;
//			vector<Point3d> object;
//			double squareSize = 50;
//			calcChessboardCorners(board_size, squareSize, object);
//
//			if (found1 == 1 && i != 0)
//			{
//				for (size_t k = 0; k<obser.size(); k++)
//				{
//					ceres::CostFunction* cost_function =
//						SnavelyReprojectionError::Create(obser[k], object[k], i, j, k);
//					problem.AddResidualBlock(cost_function,
//						NULL,
//						INN[i],
//						RR1[i],
//						TT1[i],
//						RR2[j],
//						TT2[j],
//						dis[i]
//						);
//
//				}
//			}
//			else if (found1 == 1 && i == 0)
//			{
//				for (size_t k = 0; k<obser.size(); k++)
//				{
//					ceres::CostFunction* cost_function =
//						SnavelyReprojectionError2::Create(obser[k], object[k], i, j, k);
//					problem.AddResidualBlock(cost_function,
//						NULL,
//						INN[i],
//						RR2[j],
//						TT2[j],
//						dis[i]
//						);
//				}
//			}
//
//		}
//	}
//	ceres::Solver::Options options;
//	options.linear_solver_type = ceres::DENSE_SCHUR;
//	options.minimizer_progress_to_stdout = true;
//
//	ceres::Solver::Summary summary;
//	ceres::Solve(options, &problem, &summary);
//	std::cout << summary.FullReport() << "\n";
//
//	double d[] = { 0, 0, 0, 1 };
//	string filename3 = "D:\\毕业设计\\图漾第二次标定数据\\out2\\intrinsic.xml";
//	string filename4 = "D:\\毕业设计\\图漾第二次标定数据\\out2\\ex0.xml";
//	string filename5 = "D:\\毕业设计\\图漾第二次标定数据\\out2\\exito0.xml";
//	FileStorage srin(filename3, FileStorage::WRITE);
//	FileStorage srex0(filename4, FileStorage::WRITE);
//	FileStorage srexito0(filename5, FileStorage::WRITE);
//	for (int i = 0; i<11; i++)
//	{
//		stringstream kwrite, r0write, rwrite;
//		kwrite << "K_" << i;
//		rwrite << "camera_pose_" << i;
//		r0write << "pose_timestamp_" << i;
//		Mat in1 = (Mat_<double>(3, 3) << INN[i][0], 0.0, INN[i][2], 0.0, INN[i][1], INN[i][3], 0, 0, 1);
//		//cout << in1 << endl;
//		srin << kwrite.str() << in1;
//		Mat rr1 = (Mat_<double>(3, 1) << RR1[i][0], RR1[i][1], RR1[i][2]);
//		Mat r01;
//		Rodrigues(rr1, r01);
//		//cout<<r01<<endl;
//		Mat tt1 = (Mat_<double>(3, 1) << TT1[i][0], TT1[i][1], TT1[i][2]);
//		//cout<<tt1<<endl;
//		Mat camerap = Mat::zeros(4, 4, CV_64FC1);
//		Mat rr = Mat(1, 4, CV_64FC1, d);
//		r01.copyTo(camerap(Rect(0, 0, 3, 3)));
//		tt1.copyTo(camerap(Rect(3, 0, 1, 3)));
//		rr.copyTo(camerap(Rect(0, 3, 4, 1)));
//		srexito0 << rwrite.str() << camerap;
//		//cout << camerap << endl;
//
//		Mat rr2 = (Mat_<double>(3, 1) << RR2[i][0], RR2[i][1], RR2[i][2]);
//		Mat r02;
//		Rodrigues(rr2, r02);
//		//cout<<r02<<endl;
//		Mat tt2 = (Mat_<double>(3, 1) << TT2[i][0], TT2[i][1], TT2[i][2]);
//		//cout<<tt2<<endl;
//		Mat posec = Mat::zeros(4, 4, CV_64FC1);
//		r02.copyTo(posec(Rect(0, 0, 3, 3)));
//		tt2.copyTo(posec(Rect(3, 0, 1, 3)));
//		rr.copyTo(posec(Rect(0, 3, 4, 1)));
//		srex0 << r0write.str() << posec;
//		//cout << posec << endl;
//
//		printf("%f %f\n", dis[i][0], dis[i][1]);
//		stringstream ssDIS;
//		ssDIS << "DISTORTION_" << i;
//		Mat DIS = (Mat_<float>(1, 5) << dis[i][0], dis[i][1], 0, 0, 0);
//		srin << ssDIS.str() << DIS;
//	}
//
//
//	system("pause");
//	return 0;
//}
