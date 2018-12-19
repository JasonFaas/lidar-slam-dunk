#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "StaticImageLogging.hpp"

void
StaticImageLogging::grabScreenshot(cv::Mat screen, std::string type)
{
	char buffer[80];
	getDateTime(buffer);

	std::string filename = DEV_DIRECTORY;
	filename.append(type);
	filename.append("_");
	filename.append(buffer);
	filename.append(".jpg");

	imwrite(filename, screen);
}

void
StaticImageLogging::getDateTime(char* buffer)
{
	time_t rawtime;
	struct tm * timeinfo;
	//char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, 80, "%Y_%m_%d_%H_%M_%S", timeinfo);
}

std::string
StaticImageLogging::recordingFileName(std::string type)
{
	char buffer[80];
	getDateTime(buffer);

	std::string filename = DEV_DIRECTORY;
	filename.append(type);
	filename.append("_");
	filename.append(buffer);
	filename.append(".mp4");

	return filename;
}
