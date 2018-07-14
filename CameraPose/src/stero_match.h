#pragma once


int stero_match_main( cv::Mat& leftImg,  cv::Mat& rightImg, cv::Mat& _R, cv::Mat& _t, cv::Mat& K1, cv::Mat& _D1, cv::Mat& K2, cv::Mat& _D2, std::string prefix_str, int _idx = 0);
