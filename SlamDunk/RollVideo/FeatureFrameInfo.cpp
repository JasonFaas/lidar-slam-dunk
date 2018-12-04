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

	cv::Point currRobotLocation(SimpleStaticCalc::DEPTH_WIDTH / 2, 0);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(start, end, currRobotLocation);
	onEdge = isFeatureOnEdge();

	validFeature = isValidFeature();
	if (frame == 1 && validFeature) {
		frameOneStartPoint = start;
		frameEndStartPoint = end;
		hasFrameOnePoints = true;
	}
	else {
		frameOneStartPoint = cv::Point(-1, -1);
		frameEndStartPoint = cv::Point(-1, -1);
	}
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
	return SimpleStaticCalc::isValidTriangle(startPoint, endPoint, cv::Point(SimpleStaticCalc::DEPTH_WIDTH / 2, 0));	
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
	if (!validFeature)
		throw std::invalid_argument("Invalid_Feature:FeatureFrameInfo::getPoints");
	return std::make_tuple(startPoint, endPoint);
}

std::tuple<cv::Point, cv::Point>
FeatureFrameInfo::getFrameOnePoints()
{
	if (!(validFeature && hasFrameOnePoints))
		throw std::invalid_argument("Invalid_Feature:FeatureFrameInfo::getFrameOnePoints");
	return std::make_tuple(frameOneStartPoint, frameEndStartPoint);
}

void
FeatureFrameInfo::updateFrameOnePoints(cv::Point f1StartPoint, cv::Point f1EndPoint)
{
	hasFrameOnePoints = true;
	frameOneStartPoint = f1StartPoint;
	frameEndStartPoint = f1EndPoint;
}

std::tuple<double, double>
FeatureFrameInfo::getAngles()
{
	return std::make_tuple(startPointAngle, endPointAngle);
}

int
FeatureFrameInfo::getFrame()
{
	return frame;
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

	std::cout << "FeatureFrameInfo Unit Tests Complete!" << std::endl;
	return true;
}