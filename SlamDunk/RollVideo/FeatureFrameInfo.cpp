#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "FeatureFrameInfo.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "SlamHelper.hpp"
#include "SimpleStaticCalc.hpp"


FeatureFrameInfo::FeatureFrameInfo(int currFrame, cv::Point& start, cv::Point& end)
{	
	frame = currFrame;
	startPoint = start;
	endPoint = end;

	validFeature = isValidFeature();
	if (validFeature)
		calculateInitialAngles();
	onEdge = isFeatureOnEdge();
}



bool
FeatureFrameInfo::isFeatureOnEdge()
{
	int nearEdgeMax = 10;
	std::vector<int> verifyPoints = { startPoint.x, endPoint.x };
	for (int point : verifyPoints)
	{
		if (point < nearEdgeMax || SimpleStaticCalc::DEPTH_WIDTH - point < nearEdgeMax)
			return true;
	}

	return false;
}

bool
FeatureFrameInfo::isValidFeature()
{
	validFeature = true;

	cv::Point currRobotPoint(SimpleStaticCalc::DEPTH_WIDTH / 2, 0);
	double distanceMain = SimpleStaticCalc::twoPointDistance(startPoint, endPoint);
	double distanceLeft = SimpleStaticCalc::twoPointDistance(startPoint, currRobotPoint);
	double distanceRight = SimpleStaticCalc::twoPointDistance(currRobotPoint, endPoint);
	double threeSides[3];
	threeSides[0] = distanceMain;
	threeSides[1] = distanceLeft;
	threeSides[2] = distanceRight;
	double perimeter = threeSides[2] + threeSides[1] + threeSides[0];
	for (int i = 0; i < 3; i++)
	{
		// IF a single side is longer than the other 2, recent recent frames so that feature is retired after one frame
		if (perimeter - threeSides[i] * 2 < 0)
		{
			validFeature = false;
			break;
		}
	}
}

void
FeatureFrameInfo::calculateInitialAngles()
{
	cv::Point currRobotPoint(SimpleStaticCalc::DEPTH_WIDTH / 2, 0);
	double distanceMain = SimpleStaticCalc::twoPointDistance(startPoint, endPoint);
	double distanceLeft = SimpleStaticCalc::twoPointDistance(startPoint, currRobotPoint);
	double distanceRight = SimpleStaticCalc::twoPointDistance(currRobotPoint, endPoint);
	// only calcuate angle if valid triangle
	if (validFeature)
	{
		double initialPart = (pow(distanceMain, 2) + pow(distanceLeft, 2) - pow(distanceRight, 2)) / (2 * distanceMain * distanceLeft);
		startPointAngle = acos(initialPart) * 180 / PI;

		initialPart = (pow(distanceMain, 2) + pow(distanceRight, 2) - pow(distanceLeft, 2)) / (2 * distanceMain * distanceRight);
		endPointAngle = acos(initialPart) * 180 / PI;
	}
}

bool 
FeatureFrameInfo::newFeatureCloseToThis(cv::Point& pointOne, cv::Point& pointTwo, int newFrame)
{
	// Simple invalidating conditions
	if (!validFeature || onEdge || newFrame - 1 != frame)
		return false;

	bool pointsCloseForward = SimpleStaticCalc::twoPointsClose(pointOne, startPoint) && SimpleStaticCalc::twoPointsClose(pointTwo, endPoint);
	bool pointsCloseBackward = SimpleStaticCalc::twoPointsClose(pointOne, endPoint) && SimpleStaticCalc::twoPointsClose(pointTwo, startPoint);

	return pointsCloseForward || pointsCloseForward;
}

std::tuple<cv::Point, cv::Point> 
FeatureFrameInfo::getPoints()
{
	return std::make_tuple(startPoint, endPoint);
}

bool
FeatureFrameInfo::unitTestsHere()
{

	cv::Point firstTemp(10, 10);
	cv::Point secondTemp(15, 15);
	cv::Point leftEdgeTemp(4, 25);
	cv::Point rightEdgeTemp(SimpleStaticCalc::DEPTH_WIDTH - 3, 25);

	startPoint = firstTemp;
	endPoint = leftEdgeTemp;
	if (!isFeatureOnEdge())
	{
		std::cout << "3rd" << std::endl;
		return false;
	}

	startPoint = rightEdgeTemp;
	endPoint = firstTemp;
	if (!isFeatureOnEdge())
	{
		std::cout << "4th" << std::endl;
		return false;
	}

	startPoint = firstTemp;
	endPoint = secondTemp;
	if (isFeatureOnEdge())
	{
		std::cout << "5th" << std::endl;
		return false;
	}
}