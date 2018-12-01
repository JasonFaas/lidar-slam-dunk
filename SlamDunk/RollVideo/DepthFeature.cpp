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
