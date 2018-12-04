#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class SimpleStaticCalc
{
	public:
		static double twoPointDistance(cv::Point& first, cv::Point& second);


		const static int DEPTH_WIDTH = 512;
		const static int DEPTH_HEIGHT = 424;
	private:
		
};
