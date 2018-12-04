#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "DepthFeature.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "FeatureFrameInfo.hpp"
#include "SimpleStaticCalc.hpp"


DepthFeature::DepthFeature(
	std::string name, 
	cv::Point& start, 
	cv::Point& end,
	int frame)
{
	featureName = name;

	FeatureFrameInfo firstFeatureFrameInfo(frame, start, end);
	featureFrames.push_back(firstFeatureFrameInfo);
}

// TOOD Move some unit tests to SimpleStaticCalc
bool
DepthFeature::unitTestsHere()
{
	cv::Point firstTemp(10, 10);
	cv::Point secondTemp(15, 15);
	cv::Point leftEdgeTemp(4, 25);
	cv::Point rightEdgeTemp(SimpleStaticCalc::DEPTH_WIDTH - 3, 25);


	
	
	std::cout << "\nUnit Tests Complete!\n\n" << std::endl;
	return true;
}

bool
DepthFeature::recentCloseToNewFeature(cv::Point& pointOne, cv::Point& pointTwo, int frame)
{
	if (featureFrames.size() == 0)
		return false;

	FeatureFrameInfo backElement = featureFrames.back();
	return backElement.newFeatureCloseToThis(pointOne, pointTwo, frame);
}

//void
//DepthFeature::updateRecentPoints(cv::Point& start, cv::Point& end)
//{
//	recentStartPoint = start;
//	recentEndPoint = end;
//}
//
//void
//DepthFeature::updateOriginalPoints(cv::Point& start, cv::Point& end)
//{
//	origStartPoint = start;
//	origEndPoint = end;
//}

std::tuple<cv::Point, cv::Point>
DepthFeature::getRecentPoints()
{
	FeatureFrameInfo backFeature = featureFrames.back();
	return backFeature.getPoints();
}

std::string
DepthFeature::getFeatureName()
{
	return featureName;
}

//bool
//DepthFeature::isOldFeature()
//{
//	return recentFrame != originalFrame;
//}

bool
DepthFeature::isRecentFeature()
{
	return featureFrames.size() > 1;
}

void 
DepthFeature::addNewFeatureFrame(cv::Point& pointOne, cv::Point& pointTwo, int frame)
{
	FeatureFrameInfo newestFeature(frame, pointOne, pointTwo);
	featureFrames.push_back(newestFeature);
}
