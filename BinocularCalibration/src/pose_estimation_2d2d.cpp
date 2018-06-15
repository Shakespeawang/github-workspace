
#include "binocularCalibration.h"
using namespace std;
using namespace cv;

int main ( int arfgc, char** argfv )
{
	int argc = 3;
	char * argv[] = { "", "1.png", "2.png" };
    if ( argc != 3 )
    {
        cout<<"usage: pose_estimation_2d2d img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	
	Mat R, t;
	pose_estimate(img_1, img_2, K, R, t);

	cout << "R: " << R << endl << "t:" << t << endl;
	system("pause");
    return 0;
}
