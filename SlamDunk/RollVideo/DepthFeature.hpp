#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class DepthFeature
{
	public:
		DepthFeature(std::string name, cv::Point* start, cv::Point* end, DepthFeature* left, DepthFeature* right, int frame);
		~DepthFeature();

		bool closeToExistingFeatureRecent(DepthFeature * existingFeature); // TODO Delete me after 'recentCloseToNewFeature' working
		bool recentCloseToNewFeature(cv::Point* pointOne, cv::Point* pointTwo);
		std::tuple<cv::Point *, cv::Point *> getRecentPoints();
		void updateRecentPoints(cv::Point * start, cv::Point * end);

		std::string getFeatureName();

		bool unitTestsHere();

	private:
		std::string featureName = "empty";
		cv::Point* origStartPoint = NULL;
		cv::Point* origEndPoint = NULL;
		cv::Point* recentStartPoint = NULL;
		cv::Point* recentEndPoint = NULL;
		DepthFeature* leftNeighbor = NULL;
		DepthFeature* rightNeighbor = NULL;
		int originalFrame = -1;

		bool twoPointsClose(cv::Point* first, cv::Point* second);
};
