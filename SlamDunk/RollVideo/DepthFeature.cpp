#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>


DepthFeature::DepthFeature(
	std::string name, 
	cv::Point* start, 
	cv::Point* end, 
	DepthFeature* left, 
	DepthFeature* right,
	int frame)
{
	featureName = name;

	//original and recent info initialized as same
	origStartPoint = start;
	origEndPoint = end;
	originalFrame = frame;
	
	recentStartPoint = start;
	recentEndPoint = end;
	recentFrame = frame;

	// Calculate initial angles
	cv::Point currRobotPoint = cv::Point(400 + DEPTH_WIDTH / 2, 100 + 256);
	int distanceMain = twoPointDistance(origStartPoint, origEndPoint);
	int distanceLeft = twoPointDistance(origStartPoint, &currRobotPoint);
	int distanceRight = twoPointDistance(&currRobotPoint, origEndPoint);
	int threeSides[3];
	threeSides[0] = distanceMain;
	threeSides[1] = distanceLeft;
	threeSides[2] = distanceRight;
	int perimeter = threeSides[2] + threeSides[1] + threeSides[0];
	for (int i = 0; i < 3; i++)
	{
		// IF a single side is longer than the other 2, recent recent frames so that feature is retired after one frame
		if (perimeter - threeSides[i] * 2 < 0)
		{
			recentFrame = -1;
			break;
		}
	}
	// only calcuate angle if valid triangle
	if (recentFrame != -1)
	{
		double initialPart = (pow(distanceMain, 2) + pow(distanceLeft, 2) - pow(distanceRight, 2)) / (2 * distanceMain * distanceLeft);
		origStartPointAngle = acos(initialPart) * 180 / PI;

		initialPart = (pow(distanceMain, 2) + pow(distanceRight, 2) - pow(distanceLeft, 2)) / (2 * distanceMain * distanceRight);
		origEndPointAngle = acos(initialPart) * 180 / PI;
	}

	leftNeighbor = left;
	rightNeighbor = right;
}

DepthFeature::~DepthFeature()
{

}

bool
DepthFeature::unitTestsHere()
{
	int testCount = 0;
	cv::Point firstTemp = cv::Point(10, 10);
	cv::Point secondTemp = cv::Point(15, 15);
	cv::Point thirdTemp = cv::Point(25, 25);
	cv::Point leftEdgeTemp = cv::Point(4, 25);
	cv::Point rightEdgeTemp = cv::Point(DEPTH_WIDTH - 3, 25);
	if (!twoPointsClose(&firstTemp, &secondTemp))
	{
		std::cout << "1st" << std::endl;
		return false;
	}
	if (twoPointsClose(&firstTemp, &thirdTemp))
	{
		std::cout << "2nd" << std::endl;
		return false;
	}
	updateRecentPoints(&firstTemp, &leftEdgeTemp);
	if (!featureRecentOnEdge())
	{
		std::cout << "3rd" << std::endl;
		return false;
	}
	updateRecentPoints(&rightEdgeTemp, &firstTemp);
	if (!featureRecentOnEdge())
	{
		std::cout << "4th" << std::endl;
		return false;
	}
	updateRecentPoints(&firstTemp, &secondTemp);
	if (featureRecentOnEdge())
	{
		std::cout << "5th" << std::endl;
		return false;
	}

	// TODO write test for get robot original location
	cv::Point sevenTemp = cv::Point(400, 300);
	cv::Point eightTemp = cv::Point(400, 100);
	origStartPointAngle = 60;
	origEndPointAngle = 60;
	updateRecentPoints(&sevenTemp, &eightTemp);
	updateOriginalPoints(&sevenTemp, &eightTemp);
	cv::Point robotOrigLocation1 = getOrigRobotLocationBasedOnRecentPoints();
	if (robotOrigLocation1.x != 500 || robotOrigLocation1.y != 200)
	{
		std::cout << "7th:\t" << robotOrigLocation1.x << "\tand:\t" << robotOrigLocation1.y << std::endl;
		return false;
	}


	cv::Point fiveTemp = cv::Point(0, 0);
	cv::Point sixTemp = cv::Point(0, 400);
	origStartPointAngle = 36.87;
	origEndPointAngle = 90.0;
	updateRecentPoints(&fiveTemp, &sixTemp);
	cv::Point robotOrigLocation2 = getOrigRobotLocationBasedOnRecentPoints();
	if (robotOrigLocation2.x != 400 || robotOrigLocation2.y != 300)
	{
		std::cout << "6th:\t" << robotOrigLocation2.x << "\tand:\t" << robotOrigLocation2.y << std::endl;
		return false;
	}
	
	return true;
}

bool
DepthFeature::twoPointsClose(cv::Point* first, cv::Point* second)
{
	if (first == NULL || second == NULL)
		return false;

	double distance = twoPointDistance(first, second);
	return distance < 16;
}

double
DepthFeature::twoPointDistance(cv::Point* first, cv::Point* second)
{
	return pow(pow(first->x - second->x, 2) + pow(first->y - second->y, 2), 0.5);
}

bool
DepthFeature::recentCloseToNewFeature(cv::Point* pointOne, cv::Point* pointTwo, int frame)
{
	// TODO consider retiring DepthFeature after 3 frames, not 1
	if (frame - 1 != recentFrame)
		return false;

	// TODO add edge check
	if (twoPointsClose(pointOne, recentStartPoint) && twoPointsClose(pointTwo, recentEndPoint)
		|| twoPointsClose(pointOne, recentEndPoint) && twoPointsClose(pointTwo, recentStartPoint))
	{
		recentFrame = frame;
		updateRecentPoints(pointOne, pointTwo);
		return true;
	}

	return false;
}

void
DepthFeature::updateRecentPoints(cv::Point * start, cv::Point * end)
{
	recentStartPoint = start;
	recentEndPoint = end;
}

void
DepthFeature::updateOriginalPoints(cv::Point * start, cv::Point * end)
{
	origStartPoint = start;
	origEndPoint = end;
}

std::tuple<cv::Point *, cv::Point *>
DepthFeature::getRecentPoints()
{
	return std::make_tuple(recentStartPoint, recentEndPoint);
}

std::string
DepthFeature::getFeatureName()
{
	return featureName;
}

bool
DepthFeature::isOldFeature()
{
	return recentFrame != originalFrame;
}

bool
DepthFeature::featureRecentOnEdge()
{
	int nearEdgeMax = 10;
	std::vector<int> verifyPoints = { recentStartPoint->x, recentEndPoint->x };
	for (int point : verifyPoints)
	{
		if (point < nearEdgeMax || DEPTH_WIDTH - point < nearEdgeMax)
			return true;
	}

	return false;
}

cv::Point
DepthFeature::getOrigRobotLocationBasedOnRecentPoints()
{
	double mainLength = twoPointDistance(recentEndPoint, recentStartPoint);

	double newPointAngle = 180 - origEndPointAngle - origStartPointAngle;

	double leftLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(origEndPointAngle * PI / 180);
	double rightLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(origStartPointAngle * PI / 180);

	double xCord = (pow(leftLength, 2) - pow(rightLength, 2) + pow(mainLength, 2)) / (mainLength * 2);

	double yCord = pow(pow(leftLength, 2) - pow(xCord, 2), 0.5);


	cv::Point returnPoint = cv::Point((int)std::round(xCord), (int)std::round(yCord));


	// TODO: find angle away from left to right straight that origStartPoint and origEndPoint are at
	cv::Point shiftedEndPointZero = cv::Point(origEndPoint->x - origStartPoint->x, origEndPoint->y - origStartPoint->y);
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
		rotationalAngle = acos((pow(shiftedEndPointZero.x, 2) + pow(mainLength, 2) - pow(shiftedEndPointZero.y, 2)) / (2 * shiftedEndPointZero.x * mainLength)) * 180 / PI;

	double rotationalPointX = (cos(rotationalAngle) * PI / 180) * xCord - (sin(rotationalAngle) * PI / 180) * yCord;
	double rotationalPointY = (sin(rotationalAngle) * PI / 180) * xCord + (cos(rotationalAngle) * PI / 180) * yCord;
	cv::Point returnPointRotated = cv::Point((int)std::round(rotationalPointX), (int)std::round(rotationalPointY));

	std::cout << "Simple New xCord:\t" << std::to_string(xCord) << std::endl;
	std::cout << "Simple New yCord:\t" << std::to_string(yCord) << std::endl;
	std::cout << "Rotational Angle:\t" << std::to_string((int)rotationalAngle) << std::endl; // TODO THIS IS WRONG
	std::cout << "Rotated Xcord:\t" << std::to_string(rotationalPointX) << std::endl;
	std::cout << "Rotated Ycord:\t" << std::to_string(rotationalPointY) << std::endl;
	std::cout << "Shifted xCord End:\t" << std::to_string(shiftedEndPointZero.x) << std::endl;
	std::cout << "Simple yCord End:\t" << std::to_string(shiftedEndPointZero.y) << std::endl;
	//std::cout << std::to_string(origStartPoint->x) << std::endl;
	//std::cout << std::to_string(origStartPoint->y) << std::endl;
	//std::cout << std::to_string(origEndPoint->x) << std::endl;
	//std::cout << std::to_string(origEndPoint->y) << std::endl;

	//std::cout << std::to_string(mainLength) << std::endl;

	return returnPointRotated;
}
