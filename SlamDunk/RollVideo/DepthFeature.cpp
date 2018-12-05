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
	if (frame == 1)
	{
		hasFrameOnePoints = true;
		startPointFrameOne = start;
		endPointFrameOne = end;
	}
	featureFrames.push_back(firstFeatureFrameInfo);
}

bool
DepthFeature::unitTestsHere()
{
	std::cout << "DepthFeature Unit Tests Complete!" << std::endl;
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

std::tuple<cv::Point, cv::Point>
DepthFeature::getRecentPoints()
{
	FeatureFrameInfo backFeature = featureFrames.back();
	return backFeature.getPoints();
}

std::tuple<cv::Point, cv::Point>
DepthFeature::getFrameOnePoints()
{
	// TODO add more error handling - may be valid failure
	if (!hasFrameOnePoints)
		throw std::invalid_argument("Invalid_Feature:DepthFeature::getFrameOnePoints:" + std::to_string(hasFrameOnePoints));
	return std::make_tuple(startPointFrameOne, endPointFrameOne);
}

void
DepthFeature::updateFrameOnePoints(cv::Point& f1StartPoint, cv::Point& f1EndPoint)
{
	hasFrameOnePoints = true;
	startPointFrameOne = f1StartPoint;
	endPointFrameOne = f1EndPoint;
}

std::string
DepthFeature::getFeatureName()
{
	return featureName;
}

void 
DepthFeature::addNewFeatureFrame(cv::Point& pointOne, cv::Point& pointTwo, int frame)
{
	FeatureFrameInfo newestFeature(frame, pointOne, pointTwo);
	featureFrames.push_back(newestFeature);
}

bool
DepthFeature::isCurrentAndPrevious(int currentFrame)
{
	return featureFrames.size() >= 2 && featureFrames.back().getFrame() == currentFrame && featureFrames.end()[-2].getFrame() + 1 == currentFrame;
}

bool
DepthFeature::isBrandNew(int currentFrame)
{
	return featureFrames.size() == 1 && featureFrames.back().getFrame() == currentFrame;
}

cv::Point
DepthFeature::getNewRobotLocationRelativeToPreviousLocation()
{
	FeatureFrameInfo recentInfo = featureFrames.back();
	FeatureFrameInfo originalInfo = featureFrames.end()[-2];

	cv::Point startPoint, endPoint;
	std::tie(startPoint, endPoint) = recentInfo.getPoints();
	double startPointAngle, endPointAngle;
	std::tie(startPointAngle, endPointAngle) = originalInfo.getAngles();
	cv::Point newRobotLocationPoint = SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle);

	// from angles of 

	return newRobotLocationPoint;
}
