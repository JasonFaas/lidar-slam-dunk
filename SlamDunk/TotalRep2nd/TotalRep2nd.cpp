#include <iostream>

// includes
#include <Windows.h>
#include <Shlobj.h>
#include <strsafe.h>

#include "stdafx.h"

#include <opencv2/opencv.hpp>

#include <ppl.h>

#include "RollVideo.hpp"


bool runAllUnitTestsHere();
void printMemoryUsage();
void runProgram();

const std::string DEV_DIRECTORY = "C:/Users/jason/Desktop/Code/lidar-slam-dunk/local_resources/";

const char QUIT_KEY = 'q';
const char SCREENSHOT_KEY = 's';
const char RECORD_START_KEY = 'r';
const char RECORD_END_KEY = 't';


int main()
{
	std::cout << "Hello World! Starting 2nd version of TotalRep with FOV focus" << std::endl;

	if (!runAllUnitTestsHere()) 
	{
		std::cout << "Unit test failures" << std::endl;
		std::cin.ignore();
		exit(5);
	}

	RollVideo rollThis(DEV_DIRECTORY);
	std::string fileName = "Depth_2018_12_21_11_35_47.mp4";
	rollThis.rollVideo(fileName);


	std::cin.ignore();
	exit(0);
}

bool runAllUnitTestsHere()
{
	return true;
}