
#include "cameraPose.h"
#include "stero_match.h"
#include <Windows.h>

using namespace std;
using namespace cv;


//int main(cv::Mat& img_1, cv::Mat& img_2, cv::Mat& K1, cv::Mat& D1, cv::Mat& K2, cv::Mat& D2, int _idx)
void help()
{
	cout << "usage: BinoCalib -img 4_leftIR.jpg 4_rightIR.jpg -K K1.yaml K2.yaml" << endl;
	cout << "[4_leftIR.jpg] [4_rightIR.jpg] left and right image." << endl;
	cout << "[K1.yaml] [K2.yaml] left camera and right camera intrinsic parameters." << endl << endl;
}


int main(int argc, char* argv[])
{
	/*int argc = 10;
	char * argv[] = { "", "-img",
		"../TestData/25_leftIR.jpg","../TestData/25_rightIR.jpg",
		"-K","../TestData/K.yaml",
		"-idx","25",
		"-exe","F:\\VS-workspace\\BinocularCalibration\\Release/steroMatch.exe"
	};*/

	string leftImgFile, rightImgFile, K1_file, exePath;
	string LaunchStr;
	int _idx = 0;
	
	try {
		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "-img") == 0) {
				leftImgFile = argv[++i];
				rightImgFile = argv[++i];
			}
			else if (strcmp(argv[i], "-K") == 0) {
				K1_file = argv[++i];
			}
			else if (strcmp(argv[i], "-idx") == 0) {
				_idx = atoi(argv[++i]);
			}
			else if (strcmp(argv[i], "-exe") == 0) {
				exePath = argv[++i];
			}
		}
		LaunchStr = " " + leftImgFile + " " + rightImgFile + " --max-disparity=256 --blocksize=5 -i="
			+ K1_file + " ";

		//-- 读取图像
		Mat img_1 = imread(leftImgFile, CV_LOAD_IMAGE_COLOR);
		Mat img_2 = imread(rightImgFile, CV_LOAD_IMAGE_COLOR);

		FileStorage fs1(K1_file, FileStorage::READ);

		Mat K1, K2;

		fs1["M1"] >> K1;
		fs1["M2"] >> K2;

		string tmp_leftImgFile = leftImgFile;
		string tmp_rightImgFile = rightImgFile;
		for (size_t i = 0; i < tmp_leftImgFile.size(); i++)
		{
			if ('/' == tmp_leftImgFile[i] || '\\' == tmp_leftImgFile[i])
				tmp_leftImgFile[i] = '_';
		}
		for (size_t i = 0; i < tmp_rightImgFile.size(); i++)
		{
			if ('/' == tmp_rightImgFile[i] || '\\' == tmp_rightImgFile[i])
				tmp_rightImgFile[i] = '_';
		}

		cv::Mat init_R, init_t, opt_R, opt_t;
		pose_estimate(img_1, img_2, K1, K2, init_R, init_t, opt_R, opt_t, to_string(_idx));

		FileStorage fsRT1(to_string(_idx) + ".init.yaml", FileStorage::WRITE);
		fsRT1 << "R" << init_R;
		fsRT1 << "T" << init_t;
		fsRT1.release();
		FileStorage fsRT2(to_string(_idx) + ".opt.yaml", FileStorage::WRITE);
		fsRT2 << "R" << opt_R;
		fsRT2 << "T" << opt_t;
		fsRT2.release();

		cv::Mat R1, R2, P1, P2, Q;
		cv::Mat R12, R22, P12, P22, Q2;

		cout << "K1: " << K1 << endl << "K2: " << K2 << endl << endl;
		cout << "init_R: " << init_R << endl << "init_t: " << init_t << endl << endl;
		cout << "opt_R: " << opt_R << endl << "opt_t: " << opt_t << endl << endl;


		string tmp_launch_str = LaunchStr;
		tmp_launch_str += " -e=" + to_string(_idx) + ".init.yaml -o=" + to_string(_idx) + ".init";
		
		int ret = (int)ShellExecuteA(NULL, ("open"), "steroMatch.exe", tmp_launch_str.c_str(), NULL, SW_NORMAL);//打开exe
		if (ret < 32)//检测是否指定成功
			MessageBoxA(NULL, "Shell Execute Error!", "Error", 0);

		tmp_launch_str = LaunchStr;
		tmp_launch_str += " -e=" + to_string(_idx) + ".opt.yaml -o=" + to_string(_idx) + ".opt";

		ret = (int)ShellExecuteA(NULL, ("open"), "steroMatch.exe", tmp_launch_str.c_str(), NULL, SW_NORMAL);//打开exe
		if (ret < 32)//检测是否指定成功
			MessageBoxA(NULL, "Shell Execute Error!", "Error", 0);
	}
	catch (exception e) {
		help();
		cout << e.what() << endl;
	}
	system("pause");
	return 0;
}



/*D1 = (Mat_<double>(5, 1) << 0.506622, 0.026185, -0.010587, 0.009311, -0.503029);

D2 = (Mat_<double>(5, 1) << 0.526056, 0.498828, -0.005896, 0.032702, -0.510873);

K1 = (Mat_<double>(3, 3) <<
1158.093262, 0, 657.345215,
0, 1163.105225, 494.540863,
0, 0, 1);
K2 = (Mat_<double>(3, 3) <<
1161.299316, 0, 719.604858,
0, 1162.976685, 489.890930,
0, 0, 1);
FileStorage fsK1("K1.yaml", FileStorage::WRITE);
fsK1 << "K" << K1;
fsK1 << "D" << D1;
fsK1.release();
FileStorage fsK2("K2.yaml", FileStorage::WRITE);
fsK2 << "K" << K2;
fsK2 << "D" << D2;
fsK2.release();*/
//
//int pose_estimate_main(void)
//{
//	int argc = 11;
//	char * argv[] = { "", "../Debug/kitti_left.png", "../Debug/kitti_right.png",
//		"7.070912e+02", "6.018873e+02", "7.070912e+02", "1.831104e+02",
//		"7.070912e+02", "6.018873e+02", "7.070912e+02", "1.831104e+02"
//	};
//	/*char * argv[] = { "", "../Debug/aloeL.jpg", "../Debug/aloeR.jpg",
//	"5.3480326845051309e+02", "3.3568643204394891e+02", "5.3480326845051309e+02", "2.4066183054066337e+02",
//	"5.3480326845051309e+02", "3.3568643204394891e+02", "5.3480326845051309e+02", "2.4066183054066337e+02"
//	};*/
//
//	/*char * argv[] = { "", "0-0-0.png", "0-0-2.png",
//	"707.0912", "707.0912", "601.8873", "183.1104",
//	"707.0912", "707.0912", "601.8873", "183.1104" };
//	7.070912e+02 0.000000e+00 6.018873e+02 0.000000e+00 0.000000e+00 7.070912e+02 1.831104e+02 0.000000e+00 0.
//	*/
//
//	if (argc != 11)
//	{
//		cout << "usage: pose_estimation_2d2d img1 img2 520.9 521.0 325.1 249.7 520.9 521.0 325.1 249.7" << endl;
//		return 1;
//	}
//	//-- 读取图像
//	Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
//	Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
//
//	Mat K1 = (Mat_<double>(3, 3) <<
//		atof(argv[3]), 0, atof(argv[4]),
//		0, atof(argv[5]), atof(argv[6]),
//		0, 0, 1);
//	Mat K2 = (Mat_<double>(3, 3) <<
//		atof(argv[7]), 0, atof(argv[8]),
//		0, atof(argv[9]), atof(argv[10]),
//		0, 0, 1);
//
//
//	cv::Mat init_R, init_t, opt_R, opt_t;
//	pose_estimate(img_1, img_2, K1, K2, init_R, init_t, opt_R, opt_t);
//
//	FileStorage fs("../Debug/extrinsic_init.yaml", FileStorage::WRITE);
//	fs << "R" << init_R;
//	fs << "T" << init_t;
//	fs.release();
//	FileStorage fs2("../Debug/extrinsic_opt.yaml", FileStorage::WRITE);
//	fs2 << "R" << opt_R;
//	fs2 << "T" << opt_t;
//	fs2.release();
//
//	cv::Mat dist1, dist2, R1, R2, P1, P2, Q;
//	cv::Mat R12, R22, P12, P22, Q2;
//	dist1 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
//	dist2 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
//
//	/*cv::stereoRectify(K1, dist1, K2, dist2, img_1.size(), init_R, -init_R*init_t, R1, R2, P1, P2, Q);
//	cv::Mat imgL, imgR, recImgL, recImgR;
//	cv::Mat mapx(img_1.size(), CV_64FC1);
//	cv::Mat mapy(img_1.size(), CV_64FC1);
//	cv::initUndistortRectifyMap(P1(cv::Rect(0, 0, 3, 3)), dist1, R1, P1(cv::Rect(0, 0, 3, 3)), imgL.size(), CV_32FC1, mapx, mapy);
//	cv::remap(imgL, recImgL, mapx, mapy, CV_INTER_LINEAR);
//	cv::imwrite("recConyL.png", recImgL);
//
//	cv::initUndistortRectifyMap(P2(cv::Rect(0, 0, 3, 3)), dist2, R2, P2(cv::Rect(0, 0, 3, 3)), imgL.size(), CV_32FC1, mapx, mapy);
//	cv::remap(imgR, recImgR, mapx, mapy, CV_INTER_LINEAR);
//	cv::imwrite("recConyR.png", recImgR);*/
//
//
//
//
//	cout << "K1: " << K1 << endl << "K2: " << K2 << endl << endl;
//	cout << "init_R: " << init_R << endl << "init_t: " << init_t << endl << endl;
//	cout << "opt_R: " << opt_R << endl << "opt_t: " << opt_t << endl << endl;
//	//system("pause");
//
//	stero_match_main("../Debug/extrinsic_init.yaml", "../Debug/init_disp.jpg", "../Debug/init_left.jpg", "../Debug/init_right.jpg");
//	stero_match_main("../Debug/extrinsic_opt.yaml", "../Debug/opt_disp.jpg", "../Debug/opt_left.jpg", "../Debug/opt_right.jpg");
//	system("pause");
//	return 0;
//}

