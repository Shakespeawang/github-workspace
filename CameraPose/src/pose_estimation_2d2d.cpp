
#include "cameraPose.h"
using namespace std;
using namespace cv;

int main2( int arfgc, char** argfv )
{
	int argc = 11;
	char * argv[] = { "", "../Debug/kitti_left.png", "../Debug/kitti_right.png",
	"7.070912e+02", "6.018873e+02", "7.070912e+02", "1.831104e+02",
		"7.070912e+02", "6.018873e+02", "7.070912e+02", "1.831104e+02"
	};
	/*char * argv[] = { "", "../Debug/aloeL.jpg", "../Debug/aloeR.jpg", 
	"5.3480326845051309e+02", "3.3568643204394891e+02", "5.3480326845051309e+02", "2.4066183054066337e+02",
		"5.3480326845051309e+02", "3.3568643204394891e+02", "5.3480326845051309e+02", "2.4066183054066337e+02"
	};*/
	
	/*char * argv[] = { "", "0-0-0.png", "0-0-2.png",
		"707.0912", "707.0912", "601.8873", "183.1104",
		"707.0912", "707.0912", "601.8873", "183.1104" };
		7.070912e+02 0.000000e+00 6.018873e+02 0.000000e+00 0.000000e+00 7.070912e+02 1.831104e+02 0.000000e+00 0.
		*/
	
    if ( argc != 11 )
    {
        cout<<"usage: pose_estimation_2d2d img1 img2 520.9 521.0 325.1 249.7 520.9 521.0 325.1 249.7"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

	Mat K1 = (Mat_<double>(3, 3) << 
		atof(argv[3]), 0, atof(argv[4]), 
		0, atof(argv[5]), atof(argv[6]), 
		0, 0, 1);
	Mat K2 = (Mat_<double>(3, 3) << 
		atof(argv[7]), 0, atof(argv[8]), 
		0, atof(argv[9]), atof(argv[10]), 
		0, 0, 1);

	
	cv::Mat init_R, init_t, opt_R, opt_t;
	pose_estimate(img_1, img_2, K1, K2, init_R, init_t, opt_R, opt_t);

	FileStorage fs("../Debug/extrinsic_init.yaml",FileStorage::WRITE);
	fs << "R" << init_R;
	fs << "T" << init_t;
	fs.release();
	FileStorage fs2("../Debug/extrinsic_opt.yaml", FileStorage::WRITE);
	fs2 << "R" << opt_R;
	fs2 << "T" << opt_t;
	fs2.release();

	cv::Mat dist1, dist2, R1, R2, P1, P2, Q;
	cv::Mat R12, R22, P12, P22, Q2;
	dist1 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
	dist2 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);

	/*cv::stereoRectify(K1, dist1, K2, dist2, img_1.size(), init_R, -init_R*init_t, R1, R2, P1, P2, Q);
	cv::Mat imgL, imgR, recImgL, recImgR;
	cv::Mat mapx(img_1.size(), CV_64FC1);
	cv::Mat mapy(img_1.size(), CV_64FC1);
	cv::initUndistortRectifyMap(P1(cv::Rect(0, 0, 3, 3)), dist1, R1, P1(cv::Rect(0, 0, 3, 3)), imgL.size(), CV_32FC1, mapx, mapy);
	cv::remap(imgL, recImgL, mapx, mapy, CV_INTER_LINEAR);
	cv::imwrite("recConyL.png", recImgL);

	cv::initUndistortRectifyMap(P2(cv::Rect(0, 0, 3, 3)), dist2, R2, P2(cv::Rect(0, 0, 3, 3)), imgL.size(), CV_32FC1, mapx, mapy);
	cv::remap(imgR, recImgR, mapx, mapy, CV_INTER_LINEAR);
	cv::imwrite("recConyR.png", recImgR);*/




	cout << "K1: " << K1 << endl << "K2: " << K2 << endl << endl;
	cout << "init_R: " << init_R << endl << "init_t: " << init_t << endl << endl;
	cout << "opt_R: " << opt_R << endl << "opt_t: " << opt_t << endl << endl;
	system("pause");
    return 0;
}
