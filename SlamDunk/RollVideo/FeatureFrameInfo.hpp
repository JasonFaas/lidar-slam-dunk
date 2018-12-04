#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class FeatureFrameInfo
{
	public:
		FeatureFrameInfo(int currFrame, cv::Point& start, cv::Point& end);

	private:
		int frame = -1;
		cv::Point startPoint = NULL;
		cv::Point endPoint = NULL;
		double startPointAngle = -1;
		double endPointAngle = -1;
		bool validFeature = false;
		bool onEdge = false;

		bool featureOnEdge();

		void calculateInitialAngles();

		#define PI 3.14159265
};
