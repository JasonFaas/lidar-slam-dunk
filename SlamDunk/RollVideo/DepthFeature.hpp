#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class DepthFeature
{
	public:
		DepthFeature(std::string name, cv::Point* start, cv::Point* end, DepthFeature* left, DepthFeature* right, int frame);
		~DepthFeature();

		bool recentCloseToNewFeature(cv::Point* pointOne, cv::Point* pointTwo, int frame);
		std::tuple<cv::Point *, cv::Point *> getRecentPoints();
		void updateRecentPoints(cv::Point * start, cv::Point * end);

		std::string getFeatureName();
		bool isOldFeature();
		bool featureRecentOnEdge();

		double twoPointDistance(cv::Point* first, cv::Point* second);
		cv::Point getOrigRobotLocationBasedOnRecentPoints();

		bool unitTestsHere();



	private:
		const int DEPTH_WIDTH = 512;
		const int DEPTH_HEIGHT = 424;

		std::string featureName = "empty";
		cv::Point* origStartPoint = NULL;
		cv::Point* origEndPoint = NULL;
		int originalFrame = -1;
		cv::Point* recentStartPoint = NULL;
		cv::Point* recentEndPoint = NULL;
		int recentFrame = -1;
		DepthFeature* leftNeighbor = NULL;
		DepthFeature* rightNeighbor = NULL;


		void updateOriginalPoints(cv::Point * start, cv::Point * end);


		double origStartPointAngle = -1;
		double origEndPointAngle = -1;

		bool twoPointsClose(cv::Point* first, cv::Point* second);

		#define PI 3.14159265
};
