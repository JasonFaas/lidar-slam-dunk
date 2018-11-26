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

const int COLOR_WIDTH = 1920;
const int COLOR_HEIGHT = 1080;
const int DEPTH_WIDTH = 512;
const int DEPTH_HEIGHT = 424;
const UINT DEPTH_IMG_SIZE = DEPTH_WIDTH * DEPTH_HEIGHT;
const int DEPTH_MAX_DEPTH = 4500;
const int DEPTH_MIN_DEPTH = 500;


const int MAX_WARMUP_FRAMES = 40;

const char QUIT_KEY = 'q';
const char SCREENSHOT_KEY = 's';
const std::string DEV_DIRECTORY = "C:/Users/jason/Desktop/Code/lidar-slam-dunk/local_resources/";

void printStartupInfo();
void grabScreenshot(cv::Mat colorScreen);
void deptyByteArrToMat(UINT16* pixelData, cv::Mat depthMat);

int main()
{
	printStartupInfo();

	IKinectSensor* kinectSensor = NULL;
	IMultiSourceFrameReader* reader = NULL;

	//InitializeDefaultSensor
	HRESULT hr = GetDefaultKinectSensor(&kinectSensor);
	std::cout << "HR1:\t" << SUCCEEDED(hr) << std::endl;

	hr = kinectSensor->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
		&reader
	);

	hr = kinectSensor->Open();
	std::cout << "HR2:\t" << SUCCEEDED(hr) << std::endl;

	BOOLEAN isAvailable = NULL;
	int warmup_frames = 0;
	while (warmup_frames++ < MAX_WARMUP_FRAMES)
	{
		kinectSensor->get_IsAvailable(&isAvailable);
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

	cv::Mat colorBufferMat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC4);
	cv::Mat colorSmallMat(COLOR_HEIGHT / 2, COLOR_WIDTH / 2, CV_8UC4);
	unsigned int colorBufferSize = COLOR_HEIGHT * COLOR_WIDTH * 4 * sizeof(unsigned char);
	cv::namedWindow("Kinect_Color");

	cv::Mat depthBufferMat(DEPTH_HEIGHT, DEPTH_WIDTH, CV_8UC1);
	unsigned int depthBufferSize = DEPTH_HEIGHT * DEPTH_WIDTH * 1 * sizeof(unsigned char);
	UINT16 pixelData[DEPTH_IMG_SIZE];
	cv::namedWindow("Kinect_Depth");

	IMultiSourceFrame* multiFrame = NULL;
	ColorImageFormat imageFormat = ColorImageFormat_None;

	while (warmup_frames < MAX_WARMUP_FRAMES)
	{
		multiFrame = NULL;
		hr = reader->AcquireLatestFrame(&multiFrame);

		if (SUCCEEDED(hr))
		{
			std::cout << "HR5." << warmup_frames << ":\t WarmupFrameStatus" << std::endl;

			IDepthFrame* depthFrame = NULL;
			IDepthFrameReference* depthFrameReference = NULL;
			multiFrame->get_DepthFrameReference(&depthFrameReference);
			depthFrameReference->AcquireFrame(&depthFrame);
			if (depthFrame)
			{
				hr = depthFrame->CopyFrameDataToArray(DEPTH_IMG_SIZE, pixelData);
				deptyByteArrToMat(pixelData, depthBufferMat);
				cv::imshow("Kinect_Depth", depthBufferMat);

				// Color Frame loading - this syncs the slower depth frame with the faster color frame
				IColorFrame* colorFrame = NULL;
				IColorFrameReference* colorFrameReference = NULL;
				multiFrame->get_ColorFrameReference(&colorFrameReference);
				colorFrameReference->AcquireFrame(&colorFrame);
				colorFrame->get_RawColorImageFormat(&imageFormat);
				assert(imageFormat == ColorImageFormat_Yuy2);

				hr = colorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				cv::resize(colorBufferMat, colorSmallMat, cv::Size(), 0.5, 0.5);
				cv::imshow("Kinect_Color", colorSmallMat);

				SafeRelease(colorFrame);
				SafeRelease(colorFrameReference);
			}
			else
			{
				warmup_frames++;
			}
			SafeRelease(depthFrameReference);
			SafeRelease(depthFrame);

			char waitKey = cv::waitKey(30);
			if (waitKey == QUIT_KEY)
				break;
			else if (waitKey == SCREENSHOT_KEY)
				grabScreenshot(colorSmallMat);
		}
		else
		{
			std::cout << "HR6." << warmup_frames << ":\t Waiting for Kinect to start up" << SUCCEEDED(hr) << std::endl;
		}
		SafeRelease(multiFrame);
		Sleep(200); // Sleep for Kinect frame limitations
	}

	SafeRelease(multiFrame);
	if (kinectSensor)
	{
		kinectSensor->Close();
	}
	SafeRelease(kinectSensor);

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

void deptyByteArrToMat(UINT16* pixelData, cv::Mat depthMat)
{
	int straightPixelPoint = 0;
	for (int k = 0; k < DEPTH_HEIGHT; k++) {
		for (int m = 0; m < DEPTH_WIDTH; m++) {
			depthMat.at<uchar>(k, m) = std::min(250, pixelData[straightPixelPoint] / 18);
			straightPixelPoint++;
		}
	}
}

