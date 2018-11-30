#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>


DepthFeature::DepthFeature(cv::Point* start, cv::Point* end, DepthFeature* left, DepthFeature* right)
{
	startPoint = start;
	endPoint = end;
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
DepthFeature::closeToAnotherFeature(DepthFeature * anotherFeature)
{
	cv::Point * pointOne;
	cv::Point * pointTwo;
	std::tie(pointOne, pointTwo) = anotherFeature->getPoints();

	// TODO add edge check
	if (twoPointsClose(pointOne, startPoint) && twoPointsClose(pointTwo, endPoint)
		|| twoPointsClose(pointOne, endPoint) && twoPointsClose(pointTwo, startPoint))
	{
		return true;
	}

	return false;
}

std::tuple<cv::Point *, cv::Point *>
DepthFeature::getPoints()
{
	return std::make_tuple(startPoint, endPoint);
}