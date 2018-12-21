#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class SimpleStaticCalc
{
	public:
		static double twoPointDistance(cv::Point& first, cv::Point& second);
		static bool twoPointsClose(cv::Point& first, cv::Point& second);
		static cv::Point get3rdPointLocationFrom2PointsAndAngles(cv::Point& startPoint, cv::Point& endPoint, double startPointAngle, double endPointAngle, bool relativePositiveY);
		static std::tuple<double, double> calculateInitialAnglesTo3rdPoint(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint);
		static bool isValidTriangle(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint);
		static cv::Point calculatePointsFromEstimations(std::vector<int> estimationsX, std::vector<int> estimationsY);
		static int medianFromVector(std::vector<int> values);
		static std::tuple<int, int> getFovPoint(int column, double depth);


		static bool aboveSlopeOfMainLine(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint);

		static bool unitTestsHere();

		#define PI 3.14159265

		const static int DEPTH_WIDTH = 512;
		const static int DEPTH_HEIGHT = 424;


		const static int COLOR_WIDTH = 1920;
		const static int COLOR_HEIGHT = 1080;
		const static int DEPTH_IMG_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;
		const static int DEPTH_MAX_DEPTH = 4500;
		const static int DEPTH_MIN_DEPTH = 500;

		const static int ROBOT_POS_X = DEPTH_WIDTH / 2;
		const static int ROBOT_POS_Y = 0;

		const static bool showInputsDebug = false;

		const static int tuningFeatureLengthMin = 15;
		const static int tuningValidFeatureLengthMin = 15;
		const static int tuningfeatureLookAheadMax = 3;
		const static int tuningdepthRangeAllowable = 5;
		const static int frameJumpAhead = 0;
		static constexpr double tuningAngleTooSharp = 2.0;

	private:
		
};
