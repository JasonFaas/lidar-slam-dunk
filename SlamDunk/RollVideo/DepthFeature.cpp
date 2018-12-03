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
	cv::Point currRobotPoint = cv::Point(DEPTH_WIDTH / 2, 0);
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

	if (featureRecentOnEdge())
		recentFrame = -1;

	leftNeighbor = left;
	rightNeighbor = right;

	//verify robot location on init was for test purposes
	//getOrigRobotLocationBasedOnRecentPoints();
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
	cv::Point* robotOrigLocation1 = getOrigRobotLocationBasedOnRecentPoints();
	if (robotOrigLocation1->x != 400 - 173 || robotOrigLocation1->y != 200)
	{
		std::cout << "6th:\t" << robotOrigLocation1->x << "\tand:\t" << robotOrigLocation1->y << std::endl;
		return false;
	}


	cv::Point fiveTemp = cv::Point(400, 400);
	cv::Point sixTemp = cv::Point(800, 100);
	origEndPointAngle = 36.87;
	origStartPointAngle = 90.0 - 36.87;
	updateRecentPoints(&fiveTemp, &sixTemp);
	cv::Point* robotOrigLocation2 = getOrigRobotLocationBasedOnRecentPoints();
	if (robotOrigLocation2->x != 400 || robotOrigLocation2->y != 100)
	//if (robotOrigLocation2.x != 800 || robotOrigLocation2.y != 400)
	{
		std::cout << "7th:\t" << robotOrigLocation2->x << "\tand:\t" << robotOrigLocation2->y << std::endl;
		return false;
	}
	
	std::cout << "\nUnit Tests Complete!\n\n" << std::endl;
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
DepthFeature::isRecentFeature()
{
	return recentFrame > originalFrame;
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

cv::Point*
DepthFeature::getOrigRobotLocationBasedOnRecentPoints()
{
	double mainLength = twoPointDistance(recentEndPoint, recentStartPoint);

	double newPointAngle = 180 - origEndPointAngle - origStartPointAngle;

	double leftLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(origEndPointAngle * PI / 180);
	double rightLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(origStartPointAngle * PI / 180);

	// this equation finds third point assuming mainLength is from (0,0) to (x,0) - (for most scenarios xCord is positive and yCord is negative)
	double xCord = (pow(leftLength, 2) - pow(rightLength, 2) + pow(mainLength, 2)) / (mainLength * 2);
	double yCord = -1 * pow(pow(leftLength, 2) - pow(xCord, 2), 0.5);

	// TODO: find angle away from left to right straight that origStartPoint and origEndPoint are at
	cv::Point shiftedEndPointZero = cv::Point(recentEndPoint->x - recentStartPoint->x, recentEndPoint->y - recentStartPoint->y);
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
	cv::Point* returnPointRotated = new cv::Point((int)std::round(rotationalPointX) + recentStartPoint->x, (int)std::round(rotationalPointY) + recentStartPoint->y);

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
