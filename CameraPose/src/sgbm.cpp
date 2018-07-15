//#include <opencv\cv.hpp>
//#include <opencv2\opencv.hpp>
//using namespace std;
//using namespace cv;
//
//
//int main2333(int argc, char * argv[])
//{
//	Mat left = imread("../Debug/img0_000000_10.png", IMREAD_GRAYSCALE);
//	Mat right = imread("../Debug/img1_000000_10.png", IMREAD_GRAYSCALE);
//	Mat disp;
//	int mindisparity = 0;
//	int ndisparities = 64;
//	int SADWindowSize = 11;
//	//SGBM
//	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);
//	int P1 = 8 * left.channels() * SADWindowSize* SADWindowSize;
//	int P2 = 32 * left.channels() * SADWindowSize* SADWindowSize;
//	sgbm->setP1(P1);
//	sgbm->setP2(P2);
//	sgbm->setPreFilterCap(15);
//	sgbm->setUniquenessRatio(10);
//	sgbm->setSpeckleRange(2);
//	sgbm->setSpeckleWindowSize(100);
//	sgbm->setDisp12MaxDiff(1);
//	//sgbm->setMode(cv::StereoSGBM::MODE_HH);
//	sgbm->compute(left, right, disp);
//	disp.convertTo(disp, CV_32F, 1.0 / 16);                //除以16得到真实视差值
//	Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
//	normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
//	imwrite("SGBM_DiparityMap.jpg", disp8U);
//
//	Mat deepth;
//	disp.copyTo(deepth);
//	/* 
//	P0: 7.188560e+02 0.000000e+00 6.071928e+02 0.000000e+00 
//		0.000000e+00 7.188560e+02 1.852157e+02 0.000000e+00 
//		0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
//
//	P1: 7.188560e+02 0.000000e+00 6.071928e+02 -3.861448e+02 
//		0.000000e+00 7.188560e+02 1.852157e+02 0.000000e+00 
//		0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
//
//	P2: 7.188560e+02 0.000000e+00 6.071928e+02 4.538225e+01 
//		0.000000e+00 7.188560e+02 1.852157e+02 -1.130887e-01 
//		0.000000e+00 0.000000e+00 1.000000e+00 3.779761e-03
//
//	P3: 7.188560e+02 0.000000e+00 6.071928e+02 -3.372877e+02 
//		0.000000e+00 7.188560e+02 1.852157e+02 2.369057e+00 
//		0.000000e+00 0.000000e+00 1.000000e+00 4.915215e-03
//
//	deep = b * f / disp;
//	*/
//	deepth = 3.861448e+02 * 7.188560e+02 / disp;            //除以16得到真实视差值
//	Mat deep8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
//	normalize(deepth, deep8U, 0, 255, cv::NormTypes::NORM_MINMAX, CV_8UC1);
//	imwrite("SGBM_Deepth.jpg", deep8U);
//	return 0;
//}
