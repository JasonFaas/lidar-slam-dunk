#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "SlamHelper.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "DepthFeature.hpp"

#define PI 3.14159265

SlamHelper::SlamHelper()
{
	cv::Mat depthImage, dilationDst, blackPixelsThreshold, medianBlackPixels, tempDepthImg, maskInv;
	existingFeatures = {};
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
	cv::Mat fullRepresentation = cv::Mat(cv::Size(1600, 800), CV_8UC3, cv::Scalar(0, 0, 0));

	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);
	
	int rowOfInterest = depthImage.rows / 2;
	const int featureLookAheadMax = 4;
	const int depthRangeAllowable = 4;
	const int minPointsInLine = 20;
	//std::vector<std::tuple<cv::Point, cv::Point>> traditionalFeatures = {};
	std::vector<DepthFeature*> newFeatures = {};

	int currentStartX = 0;
	int currentStartDepth = depthImage.at<uchar>(rowOfInterest, currentStartX);
	int currentDepthRef = currentStartDepth;
	for (int i = 0; i < depthImage.cols; i++)
	{
		bool continueLine = false;
		for (int m = 0; m < featureLookAheadMax; m++)
		{
			int nextDepth = depthImage.at<uchar>(rowOfInterest, i + 1 + m);
			if (abs(nextDepth - currentDepthRef) < depthRangeAllowable)
			{
				currentDepthRef = nextDepth;
				continueLine = true;
				// Advance i to currentDepthRef
				while (m > 0) {
					i++;
					m--;
				}
				break;
			}
		}

		if (!continueLine || i + 2 + featureLookAheadMax > depthImage.cols)
		{
			if (i + 1 - currentStartX >= minPointsInLine)
			{
				//traditionalFeatures.push_back(std::make_tuple(cv::Point(currentStartX, 255 - currentStartDepth), cv::Point(i, 255 - currentDepthRef)));

				cv::Point* newStartPoint = new cv::Point(currentStartX, 255 - currentStartDepth);
				cv::Point* newEndPoint = new cv::Point(i, 255 - currentDepthRef);

				std::stringstream dispText;
				dispText << std::setfill('0') << std::setw(2) << featureNameIterator++;
				DepthFeature* currentFeature = new DepthFeature(dispText.str(), newStartPoint, newEndPoint, NULL, NULL);
				newFeatures.push_back(currentFeature);
			}
			currentStartX = i + 1;
			currentStartDepth = depthImage.at<uchar>(rowOfInterest, currentStartX);
			currentDepthRef = currentStartDepth;
		}
	}


	//Link old and new features
	for (DepthFeature* newFeature : newFeatures)
	{
		bool addNewFeatureToExisting = true;
		for (DepthFeature* existingFeature : existingFeatures)
		{
			if (newFeature->closeToExistingFeatureRecent(existingFeature))
			{
				// TODO Update existing feature
				addNewFeatureToExisting = false;
				break;
			}
		}

		// Add new feature to existing features
		if (addNewFeatureToExisting)
		{
			existingFeatures.push_back(newFeature);
		}
	}


	// highlight locations of features
	cv::Point* startPoint;
	cv::Point* endPoint;
	for (DepthFeature* newFeature : newFeatures)
	{	
		std::tie(startPoint, endPoint) = newFeature->getRecentPoints();

		// Overhead Representaiton
		cv::line(overheadCopy, cv::Point(startPoint->x, startPoint->y), cv::Point(endPoint->x, endPoint->y), cv::Scalar(180), 3, 8, 0);
		cv::putText(overheadCopy, newFeature->getFeatureName(), cv::Point(startPoint->x, startPoint->y + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200));
		// Depth Representation
		cv::line(depthCopy, cv::Point(startPoint->x, rowOfInterest), cv::Point(endPoint->x, rowOfInterest), cv::Scalar(0), 3, 8, 0);
		cv::putText(depthCopy, newFeature->getFeatureName(), cv::Point(startPoint->x, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
		// Full Representation
		// TODO: This will be more complicated for actual feature tracking over multiple frames
		cv::line(fullRepresentation, cv::Point(startPoint->x + 400, startPoint->y + 100), cv::Point(endPoint->x + 400, endPoint->y + 100), cv::Scalar(180, 180, 180), 3, 8, 0);
		cv::circle(fullRepresentation, cv::Point(400 + DEPTH_WIDTH / 2, 100 + 256), 4, cv::Scalar(100, 50, 255), -1);
	}


	std::cout << "Feature Count:\t" << existingFeatures.size() << std::endl;

	cv::namedWindow("Overhead Extra");
	imshow("Overhead Extra", overheadCopy);
	cv::namedWindow("Depth Extra");
	imshow("Depth Extra", depthCopy);
	cv::namedWindow("Full Representation");
	imshow("Full Representation", fullRepresentation);

	return depthCopy;
}
