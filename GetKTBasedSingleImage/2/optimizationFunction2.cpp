#include "optimizationFunction.h"





Size getCornerPoints(
	vector < vector < vector < Point2d > > > & all_observe,
	Size board_size, string image_path, size_t camera_num, size_t image_num)
{
	cv::Size imageSize;
	for (int i = 0; i < camera_num; i++)
	{
		vector<Mat> vec_image_i = readImage(image_path,0,camera_num - 1,image_num,".tif","-");

		vector<vector<Point2d>> observe_s_j; observe_s_j.clear();

		for (int j = 0; j < image_num; j++) {
			if (0 == j)
				imageSize = vec_image_i[0].size();

			vector<Point2f> observe;
			vector<Point2d> observe2d;

			Mat result_img;
			vec_image_i[j].copyTo(result_img);

			bool found = findChessboardCorners(result_img, board_size, observe, CALIB_CB_ADAPTIVE_THRESH);
			if (!found) {
				observe.clear();
			}
			else{
				Mat view_gray;
				cvtColor(result_img, view_gray, CV_RGB2GRAY);
				find4QuadCornerSubpix(view_gray, observe, Size(5, 5));
			}
			for (size_t k = 0; k < observe.size(); k++)
			{
				observe2d.push_back(Point2d(observe[k].x, observe[k].y));
			}
			observe_s_j.push_back(observe2d);
#ifdef _DEBUG
			Mat img0;
			//vec_image_i[_j].copyTo(img0);
			//cv::cvtColor(vec_image0[_j], img0, CV_GRAY2RGB);
			cv::drawChessboardCorners(vec_image_i[j], board_size, observe, found);
			img0 = vec_image_i[j];
#endif // _DEBUG

		}
		all_observe.push_back(observe_s_j);
	}
	return imageSize;
}

vector<Mat> readImage(string path, int _i, int _j, int _k, string suffix, string connector)
{
	vector<Mat> vec;
	vec.clear();
	for (int i = _i; i <= _i; i++) {
		for (int j = _j; j <= _j; j++) {
			for (int k = 0; k < _k; k++) {
				stringstream ss;
				ss << path << i << connector << j << connector << k << suffix;
				string fn = ss.str();
				Mat image = imread(fn);
				vec.push_back(image);
			}
		}
	}
	return vec;
}
Mat readAImage(int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	Mat image = imread(filename.str());
	return image;
}

bool readParams(std::vector<cv::Mat> & v, const std::string path, size_t nums, cv::Size size, std::string tag)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < nums; i++)
	{
		std::stringstream ss;
		ss << tag << i;
		cv::Mat m(size, CV_64FC1);
		fs[ss.str()] >> m;
		assert(!m.empty());
		v.push_back(m);
	}
	fs.release();
	return true;
}

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
	Rodrigues(rvecs, mR);
	
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



