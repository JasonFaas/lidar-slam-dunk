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


	// Calculate initial angles
	calculateInitialAngles();



	onEdge = featureOnEdge();



	// TODO
	validFeature = true;
}



bool
FeatureFrameInfo::featureOnEdge()
{
	int nearEdgeMax = 10;
	std::vector<int> verifyPoints = { startPoint.x, endPoint.x };
	for (int point : verifyPoints)
	{
		if (point < nearEdgeMax || SlamHelper::DEPTH_WIDTH - point < nearEdgeMax)
			return true;
	}

	return false;
}

void
FeatureFrameInfo::calculateInitialAngles()
{
	cv::Point currRobotPoint(SlamHelper::DEPTH_WIDTH / 2, 0);
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
	// only calcuate angle if valid triangle
	if (validFeature)
	{
		double initialPart = (pow(distanceMain, 2) + pow(distanceLeft, 2) - pow(distanceRight, 2)) / (2 * distanceMain * distanceLeft);
		startPointAngle = acos(initialPart) * 180 / PI;

		initialPart = (pow(distanceMain, 2) + pow(distanceRight, 2) - pow(distanceLeft, 2)) / (2 * distanceMain * distanceRight);
		endPointAngle = acos(initialPart) * 180 / PI;
	}
}