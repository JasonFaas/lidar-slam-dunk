#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "FeatureFrameInfo.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "SlamHelper.hpp"
#include "SimpleStaticCalc.hpp"


double
SimpleStaticCalc::twoPointDistance(cv::Point& first, cv::Point& second)
{
	return pow(pow(first.x - second.x, 2) + pow(first.y - second.y, 2), 0.5);
}

bool
SimpleStaticCalc::twoPointsClose(cv::Point& first, cv::Point& second)
{
	double distance = twoPointDistance(first, second);
	return distance < 16;
}

bool
SimpleStaticCalc::unitTestsHere()
{
	cv::Point firstTemp(10, 10);
	cv::Point secondTemp(15, 15);
	cv::Point thirdTemp(25, 25);
	if (!SimpleStaticCalc::twoPointsClose(firstTemp, secondTemp))
	{
		std::cout << "1st" << std::endl;
		return false;
	}
	if (SimpleStaticCalc::twoPointsClose(firstTemp, thirdTemp))
	{
		std::cout << "2nd" << std::endl;
		return false;
	}

	cv::Point sevenTemp(400, 300);
	cv::Point eightTemp(400, 100);
	cv::Point robotOrigLocation1 = get3rdPointLocationFrom2PointsAndAngles(sevenTemp, eightTemp, 60, 60);
	if (robotOrigLocation1.x != 400 - 173 || robotOrigLocation1.y != 200)
	{
		std::cout << "6th:\t" << robotOrigLocation1.x << "\tand:\t" << robotOrigLocation1.y << std::endl;
		return false;
	}

	cv::Point fiveTemp(400, 400);
	cv::Point sixTemp(800, 100);
	cv::Point robotOrigLocation2 = get3rdPointLocationFrom2PointsAndAngles(fiveTemp, sixTemp, 90.0 - 36.87, 36.87);
	if (robotOrigLocation2.x != 400 || robotOrigLocation2.y != 100)
	{
		std::cout << "7th:\t" << robotOrigLocation2.x << "\tand:\t" << robotOrigLocation2.y << std::endl;
		return false;
	}

	return true;
}

cv::Point
SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(cv::Point& startPoint, cv::Point& endPoint, double startPointAngle, double endPointAngle)
{
	double mainLength = twoPointDistance(endPoint, startPoint);

	double newPointAngle = 180 - endPointAngle - startPointAngle;

	double leftLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(endPointAngle * PI / 180);
	double rightLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(startPointAngle * PI / 180);

	// this equation finds third point assuming mainLength is from (0,0) to (x,0) - (for most scenarios xCord is positive and yCord is negative)
	double xCord = (pow(leftLength, 2) - pow(rightLength, 2) + pow(mainLength, 2)) / (mainLength * 2);
	double yCord = -1 * pow(pow(leftLength, 2) - pow(xCord, 2), 0.5);

	cv::Point shiftedEndPointZero(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
	double rotationalAngle = -1.0;
	if (shiftedEndPointZero.x == 0 && shiftedEndPointZero.y > 0)
		rotationalAngle = 90.0;
	else if (shiftedEndPointZero.x == 0 && shiftedEndPointZero.y < 0)
		rotationalAngle = -90.0;
	else if (shiftedEndPointZero.y == 0 && shiftedEndPointZero.x > 0)
		rotationalAngle = 0.0;
	else if (shiftedEndPointZero.y == 0 && shiftedEndPointZero.x < 0)
		rotationalAngle = 180.0;
	else
	{
		rotationalAngle = acos((pow(shiftedEndPointZero.x, 2) + pow(mainLength, 2) - pow(shiftedEndPointZero.y, 2)) / (2 * shiftedEndPointZero.x * mainLength)) * 180 / PI;
		if (shiftedEndPointZero.y < 0)
			rotationalAngle *= -1;
	}

	double rotationalPointX = cos(rotationalAngle * PI / 180) * xCord - sin(rotationalAngle * PI / 180) * yCord;
	double rotationalPointY = sin(rotationalAngle * PI / 180) * xCord + cos(rotationalAngle * PI / 180) * yCord;
	cv::Point returnPointRotated((int)std::round(rotationalPointX) + startPoint.x, (int)std::round(rotationalPointY) + startPoint.y);

	// Print out info if initial robot is not close to where it should be
	//if (abs(returnPointRotated.x - DEPTH_WIDTH / 2) > 10 || abs(returnPointRotated.y) > 10)
	//{
	//	std::cout << "\n\nRecent Start x:\t" << std::to_string(recentStartPoint->x) << std::endl;
	//	std::cout << "Recent Start y:\t" << std::to_string(recentStartPoint->y) << std::endl;
	//	std::cout << "Recent End x:\t" << std::to_string(recentEndPoint->x) << std::endl;
	//	std::cout << "Recent End y:\t" << std::to_string(recentEndPoint->y) << std::endl;
	//	std::cout << "Start Angle:\t" << std::to_string(origStartPointAngle) << std::endl;
	//	std::cout << "End Angle:\t" << std::to_string(origEndPointAngle) << std::endl;
	//	std::cout << "Close to robot xCord:\t" << std::to_string(returnPointRotated.x - DEPTH_WIDTH / 2) << std::endl;
	//	std::cout << "Close to robot yCord:\t" << std::to_string(returnPointRotated.y) << std::endl;

	//	std::cout << "\nMain Length:\t" << std::to_string(mainLength) << std::endl;
	//	std::cout << "Left Length:\t" << std::to_string(leftLength) << std::endl;
	//	std::cout << "Right Length:\t" << std::to_string(rightLength) << std::endl;
	//	std::cout << "Simple New xCord:\t" << std::to_string(xCord) << std::endl;
	//	std::cout << "Simple New yCord:\t" << std::to_string(yCord) << std::endl;
	//	std::cout << "Rotational Angle:\t" << std::to_string((int)rotationalAngle) << std::endl;
	//	std::cout << "Rotated Xcord:\t" << std::to_string(rotationalPointX) << std::endl;
	//	std::cout << "Rotated Ycord:\t" << std::to_string(rotationalPointY) << std::endl;
	//	std::cout << "Shifted xCord End:\t" << std::to_string(shiftedEndPointZero.x) << std::endl;
	//	std::cout << "Shifted yCord End:\t" << std::to_string(shiftedEndPointZero.y) << std::endl;
	//	std::cout << "Final xCord:\t" << std::to_string(returnPointRotated.x) << std::endl;
	//	std::cout << "Final yCord:\t" << std::to_string(returnPointRotated.y) << std::endl;
	//}

	//std::cout << std::to_string(mainLength) << std::endl;

	return returnPointRotated;
}
