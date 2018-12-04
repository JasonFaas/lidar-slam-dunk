#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class FeatureFrameInfo
{
	public:
		FeatureFrameInfo(int currFrame, cv::Point& start, cv::Point& end);

		bool newFeatureCloseToThis(cv::Point& pointOne, cv::Point& pointTwo, int frame);
		std::tuple<cv::Point, cv::Point> getPoints();
		std::tuple<double, double> getAngles();
		int FeatureFrameInfo::getFrame();
		bool isValidFeature();

		bool unitTestsHere();
	private:
		int frame = -1;
		cv::Point startPoint = NULL;
		cv::Point endPoint = NULL;
		double startPointAngle = -1;
		double endPointAngle = -1;
		bool validFeature = false;
		bool onEdge = false;

		bool isFeatureOnEdge();
		bool validateFeature();

};
