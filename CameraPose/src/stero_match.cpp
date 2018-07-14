/*
*  stereo_match.cpp
*  calibration
*
*  Created by Victor  Eruhimov on 1/18/10.
*  Copyright 2010 Argus Corp. All rights reserved.
*
*/

#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <string>
#include <stdio.h>
#include <iostream>
#include "stero_match.h"

using namespace cv;


double getTargetValueRate(const cv::Mat& img, const double target)
{
	if (img.channels() != 1 || img.type() != CV_8UC1)
		return -1.0;

	int cnt = 0;

	for (size_t i = 0; i < img.rows; i++)
	{
		for (size_t j = 0; j < img.cols; j++)
		{
			if (std::abs(img.at<uchar>(i, j) - target) < 10e-6)
				cnt++;
		}
	}
	return 1. * cnt / (img.rows * img.cols);
}

static void print_help()
{
	printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
	printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
		"[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
		"[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

int stero_match_main(  cv::Mat& leftImg, cv::Mat& rightImg,cv::Mat& _R, cv::Mat& _t, cv::Mat& K1, cv::Mat& _D1, cv::Mat& K2, cv::Mat& _D2, std::string prefix_str, int _idx)
{
	std::string _extrinsic_filename;
	std::string _disparity_filename;
	std::string rect_left;
	std::string rect_right;

	int argc = 1;
	char* argv[] = { "" };
	std::string img1_filename = "../Debug/kitti_left.png";
	std::string img2_filename = "../Debug/kitti_right.png";
	std::string intrinsic_filename = "../Debug/intrinsics2.yml";
	std::string extrinsic_filename = _extrinsic_filename;
	std::string disparity_filename = "";
	std::string point_cloud_filename = "";

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
	int alg = STEREO_SGBM;
	int SADWindowSize, numberOfDisparities;
	bool no_display;
	float scale;

	Ptr<StereoBM> bm = StereoBM::create(16, 9);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
	cv::CommandLineParser parser(argc, argv,
		"{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{scale|1|}{i||}{e||}{o||}{p||}");
	if (parser.has("help"))
	{
		print_help();
		return 0;
	}
	//img1_filename = parser.get<std::string>(0);
	//img2_filename = parser.get<std::string>(1);
	if (parser.has("algorithm"))
	{
		std::string _alg = parser.get<std::string>("algorithm");
		alg = _alg == "bm" ? STEREO_BM :
			_alg == "sgbm" ? STEREO_SGBM :
			_alg == "hh" ? STEREO_HH :
			_alg == "var" ? STEREO_VAR :
			_alg == "sgbm3way" ? STEREO_3WAY : -1;
	}
	numberOfDisparities = 256;// parser.get<int>("max-disparity");
	SADWindowSize = 5;// parser.get<int>("blocksize");
	scale = 1.;// parser.get<float>("scale");
	no_display = parser.has("no-display");
	if (parser.has("i"))
		intrinsic_filename = parser.get<std::string>("i");
	if (parser.has("e"))
		extrinsic_filename = parser.get<std::string>("e");
	if (parser.has("o"))
		disparity_filename = parser.get<std::string>("o");
	if (parser.has("p"))
		point_cloud_filename = parser.get<std::string>("p");
	if (!parser.check())
	{
		parser.printErrors();
		return 1;
	}
	if (alg < 0)
	{
		printf("Command-line parameter error: Unknown stereo algorithm\n\n");
		print_help();
		return -1;
	}
	if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
	{
		printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
		print_help();
		return -1;
	}
	if (scale < 0)
	{
		printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
		return -1;
	}
	if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
	{
		printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
		return -1;
	}
	/*if (img1_filename.empty() || img2_filename.empty())
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		return -1;
	}
	if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		return -1;
	}

	if (extrinsic_filename.empty() && !point_cloud_filename.empty())
	{
		printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
		return -1;
	}*/

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1;// = leftImg; //imread(img1_filename, color_mode);
	Mat img2;// = rightImg; //imread(img2_filename, color_mode);
	leftImg.copyTo(img1);
	rightImg.copyTo(img2);

	if (img1.empty())
	{
		printf("Command-line parameter error: could not load the first input image file\n");
		return -1;
	}
	if (img2.empty())
	{
		printf("Command-line parameter error: could not load the second input image file\n");
		return -1;
	}

	if (scale != 1.f)
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;

	Mat result(img1.rows, img1.cols + img2.cols + 1, img1.type());
	if (!intrinsic_filename.empty())
	{
		// reading intrinsic parameters
		//FileStorage fs(intrinsic_filename, FileStorage::READ);
		//if (!fs.isOpened())
		//{
		//	printf("Failed to open file %s\n", intrinsic_filename.c_str());
		//	//return -1;
		//}

		Mat M1, D1, M2, D2;
		//fs["M1"] >> M1;
		//fs["D1"] >> D1;
		//fs["M2"] >> M2;
		//fs["D2"] >> D2;

		M1 = K1;
		M2 = K2;
		D1 = _D1;
		D2 = _D2;

		M1 *= scale;
		M2 *= scale;

		//fs.open(extrinsic_filename, FileStorage::READ);
		//if (!fs.isOpened())
		//{
		//	printf("Failed to open file %s\n", extrinsic_filename.c_str());
		//	//return -1;
		//}

		Mat R, T, R1, P1, R2, P2;
		//fs["R"] >> R;
		//fs["T"] >> T;
		R = _R;
		T = _t;

		//D1 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
		//D2 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		/*imwrite(rect_left, img1r);
		imwrite(rect_right, img2r);*/

		img1 = img1r;
		img2 = img2r;

		img1.colRange(0, img1.cols).copyTo(result.colRange(0, img1.cols));
		img2.colRange(0, img2.cols).copyTo(result.colRange(img2.cols + 1, result.cols));

		imwrite(std::to_string(_idx) + "_" + prefix_str + ".stereoRectify.jpg", result);

		cv::namedWindow(prefix_str + "_stereo_rectify", 0);
		cv::resizeWindow(prefix_str + "_stereo_rectify", result.cols / 2., result.rows / 2.);
		cv::imshow(prefix_str + "_stereo_rectify", result);
		cv::waitKey(30000);

	}

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

	bm->setROI1(roi1);
	bm->setROI2(roi2);
	bm->setPreFilterCap(31);
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setNumDisparities(numberOfDisparities);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(15);
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);

	sgbm->setPreFilterCap(63);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = img1.channels();

	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	if (alg == STEREO_HH)
		sgbm->setMode(StereoSGBM::MODE_HH);
	else if (alg == STEREO_SGBM)
		sgbm->setMode(StereoSGBM::MODE_SGBM);
	else if (alg == STEREO_3WAY)
		sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

	Mat disp, disp8;
	//Mat img1p, img2p, dispp;
	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	int64 t = getTickCount();
	if (alg == STEREO_BM)
		bm->compute(img1, img2, disp);
	else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
		sgbm->compute(img1, img2, disp);
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);
	if (!no_display)
	{
		/*namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);*/

		std::cout << "[INFO] Rate: " << getTargetValueRate(disp8, 0.) << std::endl;

		imwrite(std::to_string(_idx) + "_" + prefix_str + ".disparity.jpg", disp8);

		cv::namedWindow(prefix_str + "_disparity", 0);
		cv::resizeWindow(prefix_str + "_disparity", disp8.cols / 2., disp8.rows / 2.);
		cv::imshow(prefix_str + "_disparity", disp8);
		cv::waitKey(30000);

		printf("\n");
	}

	if (!disparity_filename.empty())
		imwrite(disparity_filename, disp8);

	if (!point_cloud_filename.empty())
	{
		printf("storing the point cloud...");
		fflush(stdout);
		Mat xyz;
		reprojectImageTo3D(disp, xyz, Q, true);
		saveXYZ(point_cloud_filename.c_str(), xyz);
		printf("\n");
	}

	return 0;
}


//
//int stero_match_main(std::string _extrinsic_filename, std::string _disparity_filename, std::string rect_left, std::string rect_right)
//{
//	int argc = 1;
//	char* argv[] = { "" };
//	std::string img1_filename = "../Debug/kitti_left.png";
//	std::string img2_filename = "../Debug/kitti_right.png";
//	std::string intrinsic_filename = "../Debug/intrinsics2.yml";
//	std::string extrinsic_filename = _extrinsic_filename;
//	std::string disparity_filename = _disparity_filename;
//	std::string point_cloud_filename = "../Debug/disparity.xyz";
//
//	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
//	int alg = STEREO_SGBM;
//	int SADWindowSize, numberOfDisparities;
//	bool no_display;
//	float scale;
//
//	Ptr<StereoBM> bm = StereoBM::create(16, 9);
//	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
//	cv::CommandLineParser parser(argc, argv,
//		"{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{scale|1|}{i||}{e||}{o||}{p||}");
//	if (parser.has("help"))
//	{
//		print_help();
//		return 0;
//	}
//	//img1_filename = parser.get<std::string>(0);
//	//img2_filename = parser.get<std::string>(1);
//	if (parser.has("algorithm"))
//	{
//		std::string _alg = parser.get<std::string>("algorithm");
//		alg = _alg == "bm" ? STEREO_BM :
//			_alg == "sgbm" ? STEREO_SGBM :
//			_alg == "hh" ? STEREO_HH :
//			_alg == "var" ? STEREO_VAR :
//			_alg == "sgbm3way" ? STEREO_3WAY : -1;
//	}
//	numberOfDisparities = 256;// parser.get<int>("max-disparity");
//	SADWindowSize = 5;// parser.get<int>("blocksize");
//	scale = 1.;// parser.get<float>("scale");
//	no_display = parser.has("no-display");
//	if (parser.has("i"))
//		intrinsic_filename = parser.get<std::string>("i");
//	if (parser.has("e"))
//		extrinsic_filename = parser.get<std::string>("e");
//	if (parser.has("o"))
//		disparity_filename = parser.get<std::string>("o");
//	if (parser.has("p"))
//		point_cloud_filename = parser.get<std::string>("p");
//	if (!parser.check())
//	{
//		parser.printErrors();
//		return 1;
//	}
//	if (alg < 0)
//	{
//		printf("Command-line parameter error: Unknown stereo algorithm\n\n");
//		print_help();
//		return -1;
//	}
//	if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
//	{
//		printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
//		print_help();
//		return -1;
//	}
//	if (scale < 0)
//	{
//		printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
//		return -1;
//	}
//	if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
//	{
//		printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
//		return -1;
//	}
//	if (img1_filename.empty() || img2_filename.empty())
//	{
//		printf("Command-line parameter error: both left and right images must be specified\n");
//		return -1;
//	}
//	if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()))
//	{
//		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
//		return -1;
//	}
//
//	if (extrinsic_filename.empty() && !point_cloud_filename.empty())
//	{
//		printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
//		return -1;
//	}
//
//	int color_mode = alg == STEREO_BM ? 0 : -1;
//	Mat img1 = imread(img1_filename, color_mode);
//	Mat img2 = imread(img2_filename, color_mode);
//
//	if (img1.empty())
//	{
//		printf("Command-line parameter error: could not load the first input image file\n");
//		return -1;
//	}
//	if (img2.empty())
//	{
//		printf("Command-line parameter error: could not load the second input image file\n");
//		return -1;
//	}
//
//	if (scale != 1.f)
//	{
//		Mat temp1, temp2;
//		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//		resize(img1, temp1, Size(), scale, scale, method);
//		img1 = temp1;
//		resize(img2, temp2, Size(), scale, scale, method);
//		img2 = temp2;
//	}
//
//	Size img_size = img1.size();
//
//	Rect roi1, roi2;
//	Mat Q;
//
//	if (!intrinsic_filename.empty())
//	{
//		// reading intrinsic parameters
//		FileStorage fs(intrinsic_filename, FileStorage::READ);
//		if (!fs.isOpened())
//		{
//			printf("Failed to open file %s\n", intrinsic_filename.c_str());
//			return -1;
//		}
//
//		Mat M1, D1, M2, D2;
//		fs["M1"] >> M1;
//		fs["D1"] >> D1;
//		fs["M2"] >> M2;
//		fs["D2"] >> D2;
//
//		M1 *= scale;
//		M2 *= scale;
//
//		fs.open(extrinsic_filename, FileStorage::READ);
//		if (!fs.isOpened())
//		{
//			printf("Failed to open file %s\n", extrinsic_filename.c_str());
//			return -1;
//		}
//
//		Mat R, T, R1, P1, R2, P2;
//		fs["R"] >> R;
//		fs["T"] >> T;
//
//		D1 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
//		D2 = (Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);
//		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
//
//		Mat map11, map12, map21, map22;
//		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
//
//		Mat img1r, img2r;
//		remap(img1, img1r, map11, map12, INTER_LINEAR);
//		remap(img2, img2r, map21, map22, INTER_LINEAR);
//
//		imwrite(rect_left, img1r);
//		imwrite(rect_right, img2r);
//
//		img1 = img1r;
//		img2 = img2r;
//	}
//
//	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;
//
//	bm->setROI1(roi1);
//	bm->setROI2(roi2);
//	bm->setPreFilterCap(31);
//	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
//	bm->setMinDisparity(0);
//	bm->setNumDisparities(numberOfDisparities);
//	bm->setTextureThreshold(10);
//	bm->setUniquenessRatio(15);
//	bm->setSpeckleWindowSize(100);
//	bm->setSpeckleRange(32);
//	bm->setDisp12MaxDiff(1);
//
//	sgbm->setPreFilterCap(63);
//	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
//	sgbm->setBlockSize(sgbmWinSize);
//
//	int cn = img1.channels();
//
//	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
//	sgbm->setMinDisparity(0);
//	sgbm->setNumDisparities(numberOfDisparities);
//	sgbm->setUniquenessRatio(10);
//	sgbm->setSpeckleWindowSize(100);
//	sgbm->setSpeckleRange(32);
//	sgbm->setDisp12MaxDiff(1);
//	if (alg == STEREO_HH)
//		sgbm->setMode(StereoSGBM::MODE_HH);
//	else if (alg == STEREO_SGBM)
//		sgbm->setMode(StereoSGBM::MODE_SGBM);
//	else if (alg == STEREO_3WAY)
//		sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
//
//	Mat disp, disp8;
//	//Mat img1p, img2p, dispp;
//	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
//
//	int64 t = getTickCount();
//	if (alg == STEREO_BM)
//		bm->compute(img1, img2, disp);
//	else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
//		sgbm->compute(img1, img2, disp);
//	t = getTickCount() - t;
//	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());
//
//	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
//	if (alg != STEREO_VAR)
//		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
//	else
//		disp.convertTo(disp8, CV_8U);
//	if (!no_display)
//	{
//		namedWindow("left", 1);
//		imshow("left", img1);
//		namedWindow("right", 1);
//		imshow("right", img2);
//		namedWindow("disparity", 0);
//		imshow("disparity", disp8);
//		printf("press any key to continue...");
//		fflush(stdout);
//		waitKey();
//		printf("\n");
//	}
//
//	if (!disparity_filename.empty())
//		imwrite(disparity_filename, disp8);
//
//	if (!point_cloud_filename.empty())
//	{
//		printf("storing the point cloud...");
//		fflush(stdout);
//		Mat xyz;
//		reprojectImageTo3D(disp, xyz, Q, true);
//		saveXYZ(point_cloud_filename.c_str(), xyz);
//		printf("\n");
//	}
//
//	return 0;
//}
//
//
