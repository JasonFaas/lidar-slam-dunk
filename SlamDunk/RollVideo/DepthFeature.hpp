#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "FeatureFrameInfo.hpp"

class DepthFeature
{
	public:
		DepthFeature(std::string name, cv::Point& start, cv::Point& end, int frame);

		bool recentCloseToNewFeature(cv::Point& pointOne, cv::Point& pointTwo, int frame);
		std::tuple<cv::Point, cv::Point> getRecentPoints();
		//void updateRecentPoints(cv::Point& start, cv::Point& end);

		void addNewFeatureFrame(cv::Point& pointOne, cv::Point& pointTwo, int frame);

		std::string getFeatureName();
		//bool isOldFeature();
		bool isRecentFeature();
		//bool featureRecentOnEdge();

		//cv::Point getOrigRobotLocationBasedOnRecentPoints();

		bool unitTestsHere();

	private:
		std::string featureName = "empty";
		std::vector<FeatureFrameInfo> featureFrames = {};
};
