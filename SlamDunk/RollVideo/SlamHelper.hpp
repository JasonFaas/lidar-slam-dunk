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

		void closeVideoWriter(); // TODO Place this in deconstructor

	private:
		std::vector<DepthFeature> existingFeatures = {};
		std::vector<cv::Point> robotHistory = {};
		void linkExistingToNewFeatures(cv::Mat& depthImageRO);
		void drawLotsOfFeaturesV1(cv::Mat& overheadCopy, cv::Mat& depthCopy);
		std::tuple<cv::Point, cv::Point> featureFrameOneGuess(cv::Point& newStartPoint, cv::Point& newEndPoint, DepthFeature& existingCurrentFeature);
		void drawNewRobotLocation();

		const cv::Size TOTAL_REP_SIZE = cv::Size(1920 / 2 + 200, 1080 / 2);
		cv::Mat totalRepSO = cv::Mat(TOTAL_REP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
		cv::Mat totalRepFov = cv::Mat(TOTAL_REP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));

		int featureNameIterator = 1;
		int frameTracker = 0;

		const int threshVal = 25;
		const int dilation_size = 20;
		const int max_dilation_iterations = 20;

		const int TOTAL_X_OFFSET = 75;
		const int TOTAL_Y_OFFSET = 100;
		const static int rowOfInterest = SimpleStaticCalc::DEPTH_HEIGHT / 2;

		void whichExistingFeaturesDoNotHaveFrameOnePoints();

		cv::Point getRobotEstimate(int historySize, int offset);

		cv::VideoWriter totalRepVideoWriter;

		void copyDepthImageToTotalRepSO(cv::Mat& depthImage);
};
