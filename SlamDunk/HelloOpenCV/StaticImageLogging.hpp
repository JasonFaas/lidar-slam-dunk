#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class StaticImageLogging
{
	public:
		static constexpr const char* DEV_DIRECTORY = "C:/Users/jason/Desktop/Code/lidar-slam-dunk/local_resources/";

		static void grabScreenshot(cv::Mat colorScreen, std::string type);
		static void getDateTime(char* buffer);
		static std::string recordingFileName(std::string type);
	private:

};
