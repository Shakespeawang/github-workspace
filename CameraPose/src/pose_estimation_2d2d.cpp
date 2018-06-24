
#include "cameraPose.h"
using namespace std;
using namespace cv;

int main ( int arfgc, char** arfgv )
{
	int argc = 11;
	char * argv[] = { "", "../Debug/img0_000000_10.png", "../Debug/img1_000000_10.png", 
	"707.0912", "707.0912", "601.8873", "183.1104",
	"707.0912", "707.0912", "601.8873", "183.1104"};    
	
    if ( argc != 11 )
    {
        cout<<"usage: pose_estimation_2d2d img1 img2 520.9 521.0 325.1 249.7 520.9 521.0 325.1 249.7"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

	Mat K1 = (Mat_<double>(3, 3) << atof(argv[3]), 0, atof(argv[4]), 0, atof(argv[5]), atof(argv[6]), 0, 0, 1);
	Mat K2 = (Mat_<double>(3, 3) << atof(argv[7]), 0, atof(argv[8]), 0, atof(argv[9]), atof(argv[10]), 0, 0, 1);
	
	cv::Mat init_R, init_t, opt_R, opt_t;
	pose_estimate(img_1, img_2, K1, K2, init_R, init_t, opt_R, opt_t);
	cout << "K1: " << K1 << endl << "K2: " << K2 << endl << endl;
	cout << "init_R: " << init_R << endl << "init_t: " << init_t << endl << endl;
	cout << "opt_R: " << opt_R << endl << "opt_t: " << opt_t << endl << endl;
	system("pause");
    return 0;
}
