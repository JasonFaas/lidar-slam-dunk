#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>


DepthFeature::DepthFeature(std::string name, cv::Point* start, cv::Point* end, DepthFeature* left, DepthFeature* right)
{
	featureName = name;
	origStartPoint = start;
	origEndPoint = end;
	recentStartPoint = start;
	recentEndPoint = end;
	leftNeighbor = left;
	rightNeighbor = right;
}

DepthFeature::~DepthFeature()
{

}

bool
DepthFeature::unitTestsHere()
{
	cv::Point firstTemp = cv::Point(10, 10);
	cv::Point secondTemp = cv::Point(15, 15);
	cv::Point thirdTemp = cv::Point(20, 20);
	if (!twoPointsClose(&firstTemp, &secondTemp))
		return false;
	if (twoPointsClose(&firstTemp, &thirdTemp))
		return false;
	
	return true;
}

bool
DepthFeature::twoPointsClose(cv::Point* first, cv::Point* second)
{
	if (first == NULL || second == NULL)
		return false;

	double distance = pow(pow(first->x - second->x, 2) + pow(first->y - second->y, 2), 0.5);

	return distance < 10;
}

bool
DepthFeature::closeToExistingFeatureRecent(DepthFeature * existingFeature)
{
	cv::Point * pointOne;
	cv::Point * pointTwo;
	std::tie(pointOne, pointTwo) = existingFeature->getRecentPoints();

	// TODO add edge check
	if (twoPointsClose(pointOne, recentStartPoint) && twoPointsClose(pointTwo, recentEndPoint)
		|| twoPointsClose(pointOne, recentEndPoint) && twoPointsClose(pointTwo, recentStartPoint))
	{
		// Update existing recentPoints and new featureName
		existingFeature->updateRecentPoints(recentStartPoint, recentEndPoint);
		featureName = existingFeature->getFeatureName();
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