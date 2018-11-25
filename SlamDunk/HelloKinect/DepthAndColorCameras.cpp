#include <iostream>

// includes
#include <Windows.h>
#include <Shlobj.h>
// Direct2D Header Files
#include <d2d1.h>
// Kinect Header files
#include <Kinect.h>
#include <strsafe.h>

#include "stdafx.h"

#include <opencv2/opencv.hpp>

//const int cColorWidth = 1920;
//const int cColorHeight = 1080;

const int MAX_WARMUP_FRAMES = 100;

const char QUIT_KEY = 'q';
const char SCREENSHOT_KEY = 's';
const std::string DEV_DIRECTORY = "C:/Users/jason/Desktop/Code/lidar-slam-dunk/local_resources/";

void printStartupInfo();
void grabScreenshot(cv::Mat colorScreen);

int main() 
{
	printStartupInfo();

	IKinectSensor* pKinectSensor = NULL;

	//InitializeDefaultSensor
	HRESULT hr = GetDefaultKinectSensor(&pKinectSensor);
	std::cout << "HR1:\t" << SUCCEEDED(hr) << std::endl;

	hr = pKinectSensor->Open();
	std::cout << "HR2:\t" << SUCCEEDED(hr) << std::endl;

	BOOLEAN isAvailable = NULL;

	int warmup_frames = 0;
	while (warmup_frames++ < MAX_WARMUP_FRAMES) 
	{
		pKinectSensor->get_IsAvailable(&isAvailable);
		if ((bool)isAvailable) 
		{
			std::cout << "HR2." << warmup_frames << ":\t" << "Available!!!" << std::endl;
			break;
		}
		else 
		{
			std::cout << "HR2." << warmup_frames << ":\t" << "Not Available" << std::endl;
			Sleep(100);
		}
	}


	IColorFrameSource* pColorFrameSource = NULL;
	hr = pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	std::cout << "HR3:\t" << SUCCEEDED(hr) << std::endl;

	IColorFrameReader* pColorFrameReader = NULL;
	hr = pColorFrameSource->OpenReader(&pColorFrameReader);
	std::cout << "HR4:\t" << SUCCEEDED(hr) << std::endl;
	//SafeRelease(pColorFrameSource); TODO release later
	bool otherBool = pColorFrameReader;
	std::cout << "Other:\t" << otherBool << std::endl;


	IFrameDescription* pFrameDescription = NULL;
	hr = pColorFrameSource->get_FrameDescription(&pFrameDescription);
	std::cout << "HR5:\t" << SUCCEEDED(hr) << std::endl;

	int nWidth = 0;
	int nHeight = 0;
	hr = pFrameDescription->get_Width(&nWidth);
	hr = pFrameDescription->get_Height(&nHeight);
	std::cout << "Heigh-Width:\t" << nWidth << "--" << nHeight << std::endl;


	cv::Mat bufferMat(nHeight, nWidth, CV_8UC4);
	cv::Mat colorMat(nHeight / 2, nWidth / 2, CV_8UC4);
	unsigned int bufferSize = nWidth * nHeight * 4 * sizeof(unsigned char);
	cv::namedWindow("Kinect_Color");

	IColorFrame* pColorFrame = NULL;
	ColorImageFormat imageFormat = ColorImageFormat_None;
	while (warmup_frames < MAX_WARMUP_FRAMES)
	{
		pColorFrame = NULL;
		hr = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		
		if (SUCCEEDED(hr)) 
		{
			pColorFrame->get_RawColorImageFormat(&imageFormat);
			assert(imageFormat == ColorImageFormat_Yuy2);
			
			hr = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			cv::resize(bufferMat, colorMat, cv::Size(), 0.5, 0.5);
			cv::imshow("Kinect_Color", colorMat);

			SafeRelease(pColorFrame);
			char waitKey = cv::waitKey(30);
			if (waitKey == QUIT_KEY)
				break;
			else if (waitKey == SCREENSHOT_KEY)
				grabScreenshot(colorMat);
			
		}
		else
		{
			std::cout << "HR6." << warmup_frames << ":\t Waiting for Kinect to start up" << SUCCEEDED(hr) << std::endl;
			warmup_frames++;
		}
		Sleep(50); // Sleep for Kinect frame limitations
	}

	if (pKinectSensor) 
	{
		pKinectSensor->Close();
	}
	SafeRelease(pKinectSensor);

	return 0;
}

void printStartupInfo()
{
	std::cout << "Hello, World Kinect Recording!" << std::endl;
	std::cout << "Hotkey reference:" << std::endl;
	std::cout << "\tQuit Program: " << QUIT_KEY << std::endl;
	std::cout << "\tSave Screenshots: " << SCREENSHOT_KEY << std::endl;
	std::cout << "\n" << std::endl;
}

void grabScreenshot(cv::Mat colorScreen)
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, 80, "%Y_%m_%d_%H_%M_%S", timeinfo);

	std::string filename = DEV_DIRECTORY;
	filename.append("Color_");
	filename.append(buffer);
	filename.append(".jpg");

	imwrite(filename, colorScreen);
}