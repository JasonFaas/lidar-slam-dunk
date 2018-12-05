#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class SimpleStaticCalc
{
	public:
		static double twoPointDistance(cv::Point& first, cv::Point& second);
		static bool twoPointsClose(cv::Point& first, cv::Point& second);
		static cv::Point get3rdPointLocationFrom2PointsAndAngles(cv::Point& startPoint, cv::Point& endPoint, double startPointAngle, double endPointAngle);
		static std::tuple<double, double> calculateInitialAnglesTo3rdPoint(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint);
		static bool isValidTriangle(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint);
		static cv::Point calculatePointsFromEstimations(std::vector<int> estimationsX, std::vector<int> estimationsY);

		static bool unitTestsHere();

		#define PI 3.14159265

		const static int DEPTH_WIDTH = 512;
		const static int DEPTH_HEIGHT = 424;


		const static int COLOR_WIDTH = 1920;
		const static int COLOR_HEIGHT = 1080;
		const static int DEPTH_IMG_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;
		const static int DEPTH_MAX_DEPTH = 4500;
		const static int DEPTH_MIN_DEPTH = 500;

		const static int defaultRobotPosX = DEPTH_WIDTH / 2;
		const static int defaultRobotPosY = 0;


	private:
		
};
