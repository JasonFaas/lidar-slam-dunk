#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "SlamHelper.hpp"
#include <math.h>
#include <stdio.h>

#define PI 3.14159265

SlamHelper::SlamHelper()
{
	cv::Mat depthImage, dilationDst, blackPixelsThreshold, medianBlackPixels, tempDepthImg, maskInv;
}

cv::Mat
SlamHelper::blurGoodDataOverBad(cv::Mat depthImage) 
{
	cv::Mat depthCopy;
	depthImage.copyTo(depthCopy);

	cv::Rect reverseBorder = cv::Rect(dilation_size, dilation_size, depthImage.cols, depthImage.rows);
	cv::copyMakeBorder(depthImage, depthCopy, dilation_size, dilation_size, dilation_size, dilation_size, cv::BORDER_REFLECT);

	cv::Mat firstBlackPixelThresh;
	cv::threshold(depthCopy, firstBlackPixelThresh, threshVal, 255, cv::THRESH_BINARY_INV);

	//add white pixels to large areas of black pixels
	int soi = 3;
	int soiMid = 1;
	int soiStep = 2;
	Concurrency::parallel_for(0, depthCopy.rows - soi, soiStep, [&](int k) {
		Concurrency::parallel_for(0, depthCopy.cols - soi, soiStep, [&](int n) {
			cv::Rect roi = cv::Rect(n + soiMid, k + soiMid, soi, soi);

			cv::Mat roiImg = depthCopy(roi);

			double min, max;
			cv::minMaxLoc(roiImg, &min, &max);
			if ((int)max < 20) {
				depthCopy.at<uchar>(k + soiMid, n + soiMid) = 255;
			}
		});
	});

	//blur zero pixel areas
	for (int i = 0; i < max_dilation_iterations; i++)
	{
		blackPixelsThreshold = NULL;
		medianBlackPixels = NULL;
		dilationDst = NULL;
		tempDepthImg = NULL;
		maskInv = NULL;

		cv::threshold(depthCopy, blackPixelsThreshold, threshVal, 255, cv::THRESH_BINARY_INV);
		cv::Mat noBorderBlackPixels = blackPixelsThreshold(reverseBorder);
		int blackPixelCount = cv::countNonZero(noBorderBlackPixels);
		if (blackPixelCount == 0)
			break;
		else if(i + 1 == max_dilation_iterations)
			std::cout << "Unable to clear all invalid data:\t" << blackPixelCount << std::endl;

		//int element_size = dilation_size * 2 + 1;
		int element_size = (i + 1) * 2 + 1;
		cv::medianBlur(depthCopy, dilationDst, element_size);
		cv::bitwise_not(blackPixelsThreshold, maskInv);
		cv::bitwise_and(depthCopy, depthCopy, tempDepthImg, maskInv);
		cv::add(tempDepthImg, dilationDst, depthCopy, blackPixelsThreshold);
	}
	std::cout << "\n\n" << std::endl;

	blackPixelsThreshold = NULL;
	medianBlackPixels = NULL;
	dilationDst = NULL;
	maskInv = NULL;
	tempDepthImg = NULL;
	cv::medianBlur(depthCopy, dilationDst, 5);
	cv::bitwise_not(firstBlackPixelThresh, maskInv);
	cv::bitwise_and(depthCopy, depthCopy, tempDepthImg, maskInv);
	cv::add(tempDepthImg, dilationDst, depthCopy, firstBlackPixelThresh);

	depthCopy = depthCopy(reverseBorder);

	return depthCopy;
}

cv::Mat
SlamHelper::depthTo2D(cv::Mat depthImage)
{
	cv::Mat returnImg = cv::Mat(255, depthImage.cols, CV_8UC1, cv::Scalar(0));

	int rowOfInterest = depthImage.rows / 2;
	Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
		returnImg.at<uchar>(255 - depthImage.at<uchar>(rowOfInterest, n), n) = 255;
	});

	return returnImg;
}

cv::Mat
SlamHelper::depthTo2DimAdjusted(cv::Mat depthImage)
{
	cv::Mat returnImg = cv::Mat(255, depthImage.cols, CV_8UC1, cv::Scalar(0));

	int rowOfInterest = depthImage.rows / 2;
	Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
		double depthAtROI = depthImage.at<uchar>(rowOfInterest, n);
		double angleToCos = abs(90.0 - 55.0 - ((double)n) / 7.252);
		double valueToSet = depthAtROI * cos(angleToCos * PI / 180.0);
		//std::cout << "what:\t" << (int)depthAtROI << "\tmorewhat:\t" << (int)angleToCos << "\tmorewhat:\t" << (int)valueToSet << std::endl;
		returnImg.at<uchar>(255 - (int)valueToSet, n) = 255;
	});

	return returnImg;
}

cv::Mat
SlamHelper::linesOnCommonFeatures(cv::Mat depthImage, cv::Mat overheadImage)
{
	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);
	int depth[512];
	
	int rowOfInterest = depthImage.rows / 2;
	int currentStartX = -1;
	int currentStartDepth = -1;
	int featureLookAheadMax = 2;
	int currentDepthRef = -1;

	std::vector<int> v = { 5, 100, 200, 355 };
	
	for (int i = 0; i < depthImage.cols; i++)
	{
		currentStartX = i;
		currentStartDepth = depthImage.at<uchar>(rowOfInterest, i);

		currentDepthRef = currentStartDepth;
		for (int k = i+1; k + featureLookAheadMax < depthImage.cols; k++, i++) 
		{
			int nextDepth = depthImage.at<uchar>(rowOfInterest, k);
			int nextNextDepth = depthImage.at<uchar>(rowOfInterest, k);
			//TODO: make these if statements a loop
			if (abs(nextDepth - currentDepthRef) < 2)
			{
				currentDepthRef = nextDepth;
				continue;
			}
			else if (abs(nextNextDepth - currentDepthRef) < 2)
			{
				currentDepthRef = nextDepth;
				k++;
				i++;
				continue;
			}
			else
			{
				//TODO: wrap up and break statement
				if (k - currentStartX > 4)
				{

				}
				break;
			}
			
		}
	}
	/*Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
		depth[n] = depthImage.at<uchar>(rowOfInterest, n);
	});*/

	//high midpoint of features
	

	v.push_back(400);
	v.push_back(450);

	cv::line(overheadCopy, cv::Point(5, 100), cv::Point(100, 105), cv::Scalar(180), 3, 8, 0);
	cv::line(overheadCopy, cv::Point(200, 200), cv::Point(250, 220), cv::Scalar(180), 3, 8, 0);
	cv::line(overheadCopy, cv::Point(400, 100), cv::Point(500, 85), cv::Scalar(180), 3, 8, 0);

	cv::line(depthCopy, cv::Point(5, rowOfInterest), cv::Point(100, rowOfInterest), cv::Scalar(0), 3, 8, 0);
	cv::putText(depthCopy, "01", cv::Point(5, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 3, cv::Scalar(0));
	cv::line(depthCopy, cv::Point(200, rowOfInterest), cv::Point(250, rowOfInterest), cv::Scalar(0), 3, 8, 0);
	cv::line(depthCopy, cv::Point(400, rowOfInterest), cv::Point(500, rowOfInterest), cv::Scalar(0), 3, 8, 0);


	cv::namedWindow("Overhead Extra");
	imshow("Overhead Extra", overheadCopy);
	cv::namedWindow("Depth Extra");
	imshow("Depth Extra", depthCopy);

	return depthCopy;
}
