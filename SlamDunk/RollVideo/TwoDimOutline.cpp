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
	cv::Mat frame, bwFrame;

	const std::string depthWindowName = "DepthVideo";

	SlamHelper* slamHelper = new SlamHelper();

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

		cv::Mat depthTo2dFlipped = slamHelper->depthTo2D(depthBlurred);
		cv::namedWindow("Depth to 2D Flipped");
		imshow("Depth to 2D Flipped", depthTo2dFlipped);

		cv::Mat depthTo2dAdjustedFlippe = slamHelper->depthTo2DimAdjusted(depthBlurred);
		cv::namedWindow("Depth to 2D Adjusted Flipped");
		imshow("Depth to 2D Adjusted Flipped", depthTo2dAdjustedFlippe);

		cv::Mat startAndEndPoints = slamHelper->linesOnCommonFeatures(depthBlurred, depthTo2dFlipped);
		

		char waitKey = cv::waitKey(0);
		if (waitKey == QUIT_KEY)
			break;
	}


	return 0;
}

bool runAllUnitTestsHere()
{
	cv::Point testFirst = cv::Point(10, 10);
	cv::Point testSecond = cv::Point(20, 20);
	DepthFeature * testDepthFeature = new DepthFeature("empty", &testFirst, &testSecond, NULL, NULL, -1);
	if (!testDepthFeature->unitTestsHere())
	{
		std::cout << "Error with DepthFeature unit tests" << std::endl;
		std::cin.ignore();
		return false;
	}
	//TODO SafeRelease(testDepthFeature);
	//TODO delete testFirst;
	//TODO delete testSecond;

	return true;
}
