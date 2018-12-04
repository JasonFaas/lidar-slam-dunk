#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"

class SlamHelper
{
	public:
		SlamHelper();

		cv::Mat blurGoodDataOverBad(cv::Mat& depthImage);
		cv::Mat depthTo2D(cv::Mat& depthImage);
		cv::Mat depthTo2DimAdjusted(cv::Mat& depthImage);
		cv::Mat linesOnCommonFeatures(cv::Mat& depthImage, cv::Mat& overheadImage);
	private:
		std::vector<DepthFeature> existingFeatures = {};
		std::vector<DepthFeature> realizeNewFeatureAndLinkExisting(cv::Mat& depthImageRO);
		void drawLotsOfFeaturesV1(std::vector<DepthFeature>& newFeatures, cv::Mat& overheadCopy, cv::Mat& depthCopy);


		cv::Mat totalRep = cv::Mat(cv::Size(1600, 800), CV_8UC3, cv::Scalar(0, 0, 0));

		int featureNameIterator = 1;
		int frameTracker = 0;

		const int threshVal = 25;
		const int dilation_size = 20;
		const int max_dilation_iterations = 20;

		const int COLOR_WIDTH = 1920;
		const int COLOR_HEIGHT = 1080;
		const int DEPTH_IMG_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;
		const int DEPTH_MAX_DEPTH = 4500;
		const int DEPTH_MIN_DEPTH = 500;

		const int rowOfInterest = DEPTH_HEIGHT / 2;

		const int TOTAL_X_OFFSET = 400;
		const int TOTAL_Y_OFFSET = 100;
};
