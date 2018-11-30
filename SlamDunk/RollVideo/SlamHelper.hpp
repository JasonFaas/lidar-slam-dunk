#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class SlamHelper
{
	public:
		SlamHelper();

		cv::Mat blurGoodDataOverBad(cv::Mat depthImage);
		cv::Mat depthTo2D(cv::Mat depthImage);
		cv::Mat depthTo2DimAdjusted(cv::Mat depthImage);
		cv::Mat linesOnCommonFeatures(cv::Mat depthImage, cv::Mat overheadImage);
	private:
		cv::Mat depthImage, dilationDst, blackPixelsThreshold, medianBlackPixels, tempDepthImg, maskInv;

		int threshVal = 25;
		int dilation_size = 20;
		int max_dilation_iterations = 20;

		const int COLOR_WIDTH = 1920;
		const int COLOR_HEIGHT = 1080;
		const int DEPTH_WIDTH = 512;
		const int DEPTH_HEIGHT = 424;
		const int DEPTH_IMG_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;
		const int DEPTH_MAX_DEPTH = 4500;
		const int DEPTH_MIN_DEPTH = 500;
};
