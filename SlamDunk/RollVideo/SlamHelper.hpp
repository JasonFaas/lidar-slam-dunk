#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"
#include "SimpleStaticCalc.hpp"

class SlamHelper
{
	public:
		SlamHelper();

		cv::Mat blurGoodDataOverBad(cv::Mat& depthImage);
		cv::Mat depthTo2D(cv::Mat& depthImage);
		cv::Mat depthTo2DimAdjusted(cv::Mat& depthImage);
		//std::vector<int> depthToVectorAdjusted(cv::Mat& depthImage);
		cv::Mat linesOnCommonFeatures(cv::Mat& depthImage, cv::Mat& overheadImage);

	private:
		std::vector<DepthFeature> existingFeatures = {};
		void linkExistingToNewFeatures(cv::Mat& depthImageRO);
		void drawLotsOfFeaturesV1(cv::Mat& overheadCopy, cv::Mat& depthCopy);
		std::tuple<cv::Point, cv::Point> featureFrameOneGuess(cv::Point& newStartPoint, cv::Point& newEndPoint, DepthFeature& existingCurrentFeature);
		void drawNewRobotLocation();

		cv::Mat totalRep = cv::Mat(cv::Size(1600, 800), CV_8UC3, cv::Scalar(0, 0, 0));

		int featureNameIterator = 1;
		int frameTracker = 0;

		const int threshVal = 25;
		const int dilation_size = 20;
		const int max_dilation_iterations = 20;

		const int TOTAL_X_OFFSET = 400;
		const int TOTAL_Y_OFFSET = 400;
		const static int rowOfInterest = SimpleStaticCalc::DEPTH_HEIGHT / 2;

		void whichExistingFeaturesDoNotHaveFrameOnePoints();
};
