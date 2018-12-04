#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "FeatureFrameInfo.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "SlamHelper.hpp"
#include "SimpleStaticCalc.hpp"


double
SimpleStaticCalc::twoPointDistance(cv::Point& first, cv::Point& second)
{
	return pow(pow(first.x - second.x, 2) + pow(first.y - second.y, 2), 0.5);
}
