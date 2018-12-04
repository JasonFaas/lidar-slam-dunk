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

		void addNewFeatureFrame(cv::Point& pointOne, cv::Point& pointTwo, int frame);
		std::string getFeatureName();
		bool isCurrentAndPrevious(int currentFrame);
		bool isBrandNew(int currentFrame);

		cv::Point getNewRobotLocationRelativeToPreviousLocation();

		bool unitTestsHere();

	private:
		std::string featureName = "empty";
		std::vector<FeatureFrameInfo> featureFrames = {};
};
