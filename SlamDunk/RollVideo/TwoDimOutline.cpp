#include <iostream>

// includes
#include <Windows.h>
#include <Shlobj.h>
#include <strsafe.h>

#include "stdafx.h"

#include <opencv2/opencv.hpp>

#include <ppl.h>
#include "DepthFeature.hpp"

#include "SlamHelper.hpp"
#include "FeatureFrameInfo.hpp"

bool runAllUnitTestsHere();

const std::string DEV_DIRECTORY = "C:/Users/jason/Desktop/Code/lidar-slam-dunk/local_resources/";

const char QUIT_KEY = 'q';
const char SCREENSHOT_KEY = 's';
const char RECORD_START_KEY = 'r';
const char RECORD_END_KEY = 't';

int main()
{
	if (!runAllUnitTestsHere())
		return -1;


	cv::VideoCapture capture(DEV_DIRECTORY + "Depth_2018_11_26_10_46_04.mp4");
	//cv::VideoCapture capture(DEV_DIRECTORY + "Depth_2018_12_05_12_57_08.mp4");
	//cv::VideoCapture capture(DEV_DIRECTORY + "Depth_2018_12_05_13_03_41.mp4");
	cv::Mat frame, bwFrame;

	const std::string depthWindowName = "DepthVideo";

	SlamHelper* slamHelper = new SlamHelper();

	int frameTracker = 0;
	cv::namedWindow(depthWindowName);
	while (TRUE)
	{
		capture >> frame;
		if (frame.empty())
			break;

		cv::cvtColor(frame, bwFrame, cv::COLOR_BGR2GRAY);

		cv::imshow(depthWindowName, bwFrame);

		cv::Mat depthBlurred = slamHelper->blurGoodDataOverBad(bwFrame);
		cv::namedWindow("Bad Data Blurred Out");
		imshow("Bad Data Blurred Out", depthBlurred);

		cv::Mat depthTo2d = slamHelper->depthTo2D(depthBlurred);
		cv::namedWindow("Depth to 2D");
		imshow("Depth to 2D", depthTo2d);

		cv::Mat depthTo2dAdjusted = slamHelper->depthTo2DimAdjusted(depthBlurred);
		cv::namedWindow("Depth to 2D Adjusted");
		imshow("Depth to 2D Adjusted", depthTo2dAdjusted);

		//std::vector<int> depthTo2dVector = slamHelper->depthToVectorAdjusted(depthBlurred);

		cv::Mat startAndEndPoints = slamHelper->linesOnCommonFeatures(depthBlurred, depthTo2d);

		
		if (frameTracker++ > SimpleStaticCalc::frameJumpAhead) {
			char waitKey = cv::waitKey(0);
			if (waitKey == QUIT_KEY) {
				slamHelper->closeVideoWriter();
				break;
			}
		}
	}


	return 0;
}

bool runAllUnitTestsHere()
{
	cv::Point testFirst(10, 10);
	cv::Point testSecond(20, 20);
	DepthFeature testDepthFeature("empty", testFirst, testSecond, -1);
	if (!testDepthFeature.unitTestsHere())
	{
		std::cout << "Error with DepthFeature unit tests" << std::endl;
		std::cin.ignore();
		return false;
	}
	if (!SimpleStaticCalc::unitTestsHere())
	{
		std::cout << "Error with SimpleStaticCalc unit tests" << std::endl;
		std::cin.ignore();
		return false;
	}
	FeatureFrameInfo framedUnitTests(-1, cv::Point(100, 100), cv::Point(400, 400));
	if (!framedUnitTests.unitTestsHere())
	{
		std::cout << "Error with FeatureFrameInfo unit tests" << std::endl;
		std::cin.ignore();
		return false;
	}

	return true;
}
