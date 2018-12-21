#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class StaticImageLogging
{
	public:

		static void grabScreenshot(cv::Mat colorScreen, std::string type);
		static void getDateTime(char* buffer);
		static std::string recordingFileName(std::string type);
	private:
		static std::string getDevDirectory();
};
