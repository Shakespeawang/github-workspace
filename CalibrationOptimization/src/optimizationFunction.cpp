#include "optimizationFunction.h"





bool get_s0_s1(vector<double> & s_0, vector<double> & s_1,
	vector<Point3f> observe0, vector<Point3f> observe1,
	Mat K0, Mat K1,
	Mat camera_pos
	)
{
	s_0.clear();
	s_1.clear();
	if (observe0.size() != observe1.size() || observe0.size() <= 0 || observe1.size() <= 0)
		return false;
	for (int j = 0; j < observe0.size(); j++) {
		Mat position0(3, 1, CV_32FC1);
		Mat observe0_point(3, 1, CV_32FC1);
		observe0_point.at<float>(0, 0) = observe0[j].x;
		observe0_point.at<float>(1, 0) = observe0[j].y;
		observe0_point.at<float>(2, 0) = 1.0f;
		position0 = K0.inv() * observe0_point;

		Mat position1(3, 1, CV_32FC1);
		Mat observe1_point(3, 1, CV_32FC1);
		observe1_point.at<float>(0, 0) = observe1[j].x;
		observe1_point.at<float>(1, 0) = observe1[j].y;
		observe1_point.at<float>(2, 0) = 1.0f;
		position1 = K1.inv() * observe1_point;

		double s0, s1, s01, s02, s11, s12;
		double x0 = position0.at<float>(0, 0);
		double y0 = position0.at<float>(1, 0);
		double z0 = position0.at<float>(2, 0);
		double x1 = position1.at<float>(0, 0);
		double y1 = position1.at<float>(1, 0);
		double z1 = position1.at<float>(1, 0);

		float **arr = new float *[camera_pos.rows];
		for (int j = 0; j < camera_pos.rows; ++j)
			arr[j] = camera_pos.ptr<float>(j);

#ifdef _DEBUG
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				//fcout << arr[i][j] << "\t";
			}
			//fcout << endl;
		}
		Mat _D_01 = camera_pos;
#endif // _DEBUG


		double alpha = arr[0][0] * x0 + arr[0][1] * y0 + arr[0][2] * z0;
		double beta = arr[1][0] * x0 + arr[1][1] * y0 + arr[1][2] * z0;
		double gama = arr[2][0] * x0 + arr[2][1] * y0 + arr[2][2] * z0;
		try {
			s0 = (x1 / y1 * arr[1][3] - arr[0][3]) / (alpha - beta * x1 / y1);
			s1 = s0 * gama + arr[2][3];
		}
		catch (exception e) {
			continue;
		}
		s_0.push_back(s0);
		s_1.push_back(s1);
	}
	return true;
}



bool out_all_point(const vector < vector < vector < Point3f > > > & all_observe, const string ss1)
{
	fstream fs1(ss1, ios::out);
	if (!fs1.is_open())
		return false;

	int point_cnt = 0;
	int flag_cnt = 0;
	for (int _i = 0; _i < all_observe.size(); _i++)
	{
		fs1 << "\n\n\n\n\n第" << _i << "个相机：\n";

		for (int _j = 0; _j < all_observe[_i].size(); _j++) {

			fs1 << "\n\t第" << _j << "张图片：\n";

			for (int k = 0; k < all_observe[_i][_j].size(); k++) {
				point_cnt++;
				bool flag = all_observe[_i][_j][k].z;
				if (!flag)
					flag_cnt++;//如果flag为false,则记录不参与计算的点的个数
				fs1 << "\t" << k << ",\tx: " << all_observe[_i][_j][k].x
					<< ",\ty: " << all_observe[_i][_j][k].y
					<< ",\tz(flag): " << all_observe[_i][_j][k].z << endl;
			}

		}
	}
	fs1 << "\n\n总共: " << point_cnt << "点\n";
	fs1 << "\n\n不参与计算的有: " << flag_cnt << "点\n";
	fs1 << "\n\n参与计算的有: " << (point_cnt - flag_cnt) << "点\n";
	return true;
}



void output_image_point_cloud(
	const vector < vector < vector < Point3f > > > & all_observe,
	const vector<Mat_<float>> intrinsic, const vector<Mat_<float>> camera_pos, const vector<Mat_<float>> pose_timestamp,
	const vector<vector<vector<Point3d> > > recon_pts,
	const string ss1,
	const bool is_have_si
	)
{
	for (int Camera_i = 0; Camera_i < all_observe.size(); Camera_i++) {

		for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++) {

			stringstream ss;
			ss << ss1 << "__camera_" << Camera_i << "__image_" << image_j << ".txt";

			fstream fs1(ss.str(), ios::out);

			Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			for (int i = 0; i < 4; i++) {
				float * ptr = tmp_intrinsic_i.ptr<float>(i);
				for (int j = 0; j < 4; j++) {
					if (i < 3 && j < 3)
						ptr[j] = intrinsic[Camera_i].at<float>(i, j);
					else if (3 == i && 3 == j)
						ptr[j] = 1.0;
					else
						ptr[j] = 0.0;
				}
			}

			Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo = tmp_intrinsic_i * camera_pos[Camera_i];
			Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo_inv = homo.inv();

			for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++) {

				//反向投影
				if (is_have_si) {
					if (recon_pts[Camera_i][image_j].size() <= 0) {
						continue;
					}
				}
				bool flag = all_observe[Camera_i][image_j][k].z;//是否参与计算
				if (flag) {
					Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					tmp_image_point.at<float>(0, 0) = recon_pts[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].x;
					tmp_image_point.at<float>(1, 0) = recon_pts[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].y;
					tmp_image_point.at<float>(2, 0) = recon_pts[Camera_i][image_j][k].z;
					tmp_image_point.at<float>(3, 0) = 1.0;

					Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
					possible_object_point = homo_inv * tmp_image_point;

					Point3d object_point;
					object_point.x = possible_object_point.at<float>(0, 0);
					object_point.y = possible_object_point.at<float>(1, 0);
					object_point.z = possible_object_point.at<float>(2, 0);


					fs1 << object_point.x << " " << object_point.y << " " << object_point.z << endl;
				}
			}
			fs1.close();
		}
	}

}



vector<Mat> readImage(string path, int _k, int _j, int _i, string suffix, string connector)
{
	vector<Mat> vec;
	vec.clear();
	for (int i = _i; i <= _i; i++) {
		for (int j = _j; j <= _j; j++) {
			for (int k = 0; k < _k; k++) {
				stringstream filename;
				filename << path << i << connector << j << connector << k << suffix;
				Mat image = imread(filename.str());
				vec.push_back(image);
			}
		}
	}
	return vec;
}
Mat readImage(int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	Mat image = imread(filename.str());
	return image;
}
bool writeImage(Mat image, int _i, int _j, int _k, string path, string suffix, string connector)
{
	stringstream filename;
	filename << path << _i << connector << _j << connector << _k << suffix;
	return imwrite(filename.str(), image);
}


 bool writeParams(std::string path, std::vector<cv::Mat_<float>> & vec1, std::string flag1, std::vector<cv::Mat_<float>> & vec2, std::string flag2)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < vec1.size(); i++)
	{
		std::stringstream ss1;
		ss1 << flag1 << i;
		fs << ss1.str() << vec1[i];
	}
	for (int i = 0; i < vec2.size(); i++)
	{
		std::stringstream ss2;
		ss2 << flag2 << i;
		fs << ss2.str() << vec2[i];
	}
	fs.release();
	return true;
}
 bool readParams(std::string path, int nums, std::vector<cv::Mat_<float>> & vec1, cv::Size Size1, std::string flag1, std::vector<cv::Mat_<float>> & vec2, cv::Size Size2, std::string flag2)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	for (int i = 0; i < nums; i++)
	{
		if (flag2.empty()){
			std::stringstream ss1;
			ss1 << flag1 << i;
			cv::Mat mat1(Size1, CV_32FC1);
			fs[ss1.str()] >> mat1;
			if (!mat1.empty())
				vec1.push_back(mat1);
		}
		else{
			std::stringstream ss1, ss2;
			ss1 << flag1 << i;
			ss2 << flag2 << i;
			cv::Mat mat1(Size1, CV_32FC1);
			cv::Mat mat2(Size2, CV_32FC1);
			fs[ss1.str()] >> mat1;
			fs[ss2.str()] >> mat2;
			if (!mat1.empty())
				vec1.push_back(mat1);
			if (!mat2.empty())
				vec2.push_back(mat2);
		}
	}
	fs.release();
	return true;
}


 void transform2RT(const cv::Mat_<float> & T, double R[], double t[])
{

	cv::Mat m_T = T(cv::Rect(0, 0, 3, 3));
	cv::Mat m_R = cv::Mat(3, 1, CV_64FC1);
	Rodrigues(m_T, m_R);
	R[0] = m_R.at<float>(0, 0);
	R[1] = m_R.at<float>(1, 0);
	R[2] = m_R.at<float>(2, 0);


	cv::Mat m_t = cv::Mat(3, 1, CV_64FC1);
	T(cv::Rect(3, 0, 1, 3)).copyTo(m_t.rowRange(0, 3));
	t[0] = m_t.at<double>(0, 0);
	t[1] = m_t.at<double>(1, 0);
	t[2] = m_t.at<double>(2, 0);
}
 void RT2transform(const double R[], const double t[], cv::Mat_<float> & T)
{
	cv::Mat rvecs = (cv::Mat_<float>(3, 1) << R[0], R[1], R[2]); // 旋转向量
	cv::Mat mR;          // 图像的旋转矩阵 ;

	// 将旋转向量转换为相对应的旋转矩阵 */   
	Rodrigues(rvecs, mR);
	
	T = (cv::Mat_<float>(4, 4) <<
		mR.at<float>(0, 0), mR.at<float>(0, 1), mR.at<float>(0, 2), t[0],
		mR.at<float>(1, 0), mR.at<float>(1, 1), mR.at<float>(1, 2), t[1],
		mR.at<float>(2, 0), mR.at<float>(2, 1), mR.at<float>(2, 2), t[2],
		0, 0, 0, 1);
}


 void camera_2_array(const cv::Mat_<float> K, double arr[])
{
	arr[0] = K.at<float>(0, 0);
	arr[1] = K.at<float>(1, 1);
	arr[2] = K.at<float>(0, 2);
	arr[3] = K.at<float>(1, 2);
}
void array_2_camera(const double arr[], cv::Mat_<float> & K)
{
	float arr2[] = { arr[0], arr[1], arr[2], arr[3] };
	K = (cv::Mat_<float>(3, 3) <<
		arr2[0], 0, arr2[2],
		0, arr2[1], arr2[3],
		0, 0, 1);
}


 bool matrix_2_array(const cv::Mat_<float> mat, double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < arr_row; i++) {
		for (int j = 0; j < arr_col; j++) {
			if (i * arr_col + j < arr_row * arr_col) {
				if (i < mat.rows && j < mat.cols)
					arr[i * arr_col + j] = mat.at<float>(i, j);
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
 bool array_2_matrix(cv::Mat_<float> & mat, const double arr[], const int arr_row, const int arr_col)
{
	//system("cls");
	for (int i = 0; i < mat.rows; i++) {
		for (int j = 0; j < mat.cols; j++) {
			if (i < mat.rows && j < mat.cols) {
				if (i * arr_col + j < arr_row * arr_col)
					mat.at<float>(i, j) = arr[i * arr_col + j];
				else if (i == j)
					mat.at<float>(i, j) = 1.0;
				else
					mat.at<float>(i, j) = 0.0;
			}
			//cout << mat.at<float>(i, j) << "\t";
		}
		//cout << "\n";
	}
	return true;
}



vector<Point3d> optimize_array(const std::vector<vector<Point3d>> & arr)
{
	vector<Point3d> sum;
	sum.clear();

	for (int i = 0; i < arr.size(); i++) {
		for (int j = 0; j < arr[i].size(); j++){
			Point3d p(0, 0, 0);
			sum.push_back(p);
		}
		if (arr[i].size() > 0)
			break;
	}
	int cnt = 0;
	for (int i = 0; i < arr.size(); i++){
		if (arr[i].size() > 0)
			cnt++;
		for (int j = 0; j < arr[i].size(); j++)
		{
			sum[j].x += arr[i][j].x;
			sum[j].y += arr[i][j].y;
			sum[j].z += arr[i][j].z;
		}
	}

	for (int i = 0; i < sum.size(); i++) {
		if (cnt != 0)
			sum[i].z /= cnt;
		else
			sum[i].z = 0.0;
	}
	return sum;
}
bool sort_min(cv::Point2f p1, cv::Point2f p2)
{
	return p1.x < p2.x;
}
void optimize_all_observe(
	std::vector <std::vector <std::vector <cv::Point3f > > > & all_observe,
	const cv::vector<cv::Mat_<float>> intrinsic, const cv::vector<cv::Mat_<float>> camera_pos, const cv::vector<cv::Mat_<float>> pose_timestamp,
	const cv::vector<cv::vector<cv::vector<Point3d> > > S_Si, const cv::vector<cv::Point3d> object, const cv::Size board_size
	)
{
	for (int Camera_i = 0; Camera_i < all_observe.size(); Camera_i++) {

		for (int image_j = 0; image_j < all_observe[Camera_i].size(); image_j++) {

			cv::Mat tmp_intrinsic_i(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			for (int i = 0; i < 4; i++) {
				float * ptr = tmp_intrinsic_i.ptr<float>(i);
				for (int j = 0; j < 4; j++) {
					if (i < 3 && j < 3)
						ptr[j] = intrinsic[Camera_i].at<float>(i, j);
					else if (3 == i && 3 == j)
						ptr[j] = 1.0;
					else
						ptr[j] = 0.0;
				}
			}

			cv::Mat homo(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo = tmp_intrinsic_i * camera_pos[Camera_i] * pose_timestamp[image_j];
			cv::Mat homo_inv(4, 4, CV_MAKE_TYPE(CV_32FC1, 1));
			homo_inv = homo.inv();

			double sum_loss3 = 0.0, half_sum_loss3 = 0.0;

			if (S_Si[Camera_i][image_j].size() <= 0) {
				continue;
			}
			cv::vector<cv::Point2d > error_sum;
			error_sum.clear();
			for (int k = 0; k < all_observe[Camera_i][image_j].size(); k++) {


				//反向投影
				cv::Mat tmp_image_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				tmp_image_point.at<float>(0, 0) = S_Si[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].x;
				tmp_image_point.at<float>(1, 0) = S_Si[Camera_i][image_j][k].z * all_observe[Camera_i][image_j][k].y;
				tmp_image_point.at<float>(2, 0) = S_Si[Camera_i][image_j][k].z;
				tmp_image_point.at<float>(3, 0) = 1.0;

				cv::Mat possible_object_point(4, 1, CV_MAKE_TYPE(CV_32FC1, 1));
				possible_object_point = homo_inv * tmp_image_point;

				cv::Point3d object_point;
				object_point.x = possible_object_point.at<float>(0, 0);
				object_point.y = possible_object_point.at<float>(1, 0);
				object_point.z = possible_object_point.at<float>(2, 0);

				sum_loss3 = pow(object_point.x - object[k].x, 2)
					+ pow(object_point.y - object[k].y, 2)
					+ pow(object_point.z - object[k].z, 2);

				cv::Point2d pnt(sum_loss3, k);

				error_sum.push_back(pnt);
			}
			sort(error_sum.begin(), error_sum.end(), sort_min);

			for (int k = error_sum.size() - 1; k >= 0 && k > error_sum.size() - 1 - board_size.height; --k) {
				int idx = (int)error_sum[k].y;
				all_observe[Camera_i][image_j][idx].z = false;
			}

		}
	}
}


