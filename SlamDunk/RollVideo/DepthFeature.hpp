#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class DepthFeature
{
	public:
		DepthFeature(cv::Point* start, cv::Point* end, DepthFeature* left, DepthFeature* right);
		~DepthFeature();

		bool closeToAnotherFeature(DepthFeature * anotherFeature);
		std::tuple<cv::Point *, cv::Point *> getPoints();

		bool unitTestsHere();

	private:
		cv::Point* startPoint = NULL;
		cv::Point* endPoint = NULL;
		DepthFeature* leftNeighbor = NULL;
		DepthFeature* rightNeighbor = NULL;

		bool twoPointsClose(cv::Point* first, cv::Point* second);
};
