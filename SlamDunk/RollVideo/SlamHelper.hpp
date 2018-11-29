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
	private:
		cv::Mat depthImage, dilationDst, blackPixelsThreshold, medianBlackPixels, tempDepthImg, maskInv;

		int threshVal = 25;
		int dilation_size = 20;
		int max_dilation_iterations = 20;
};
