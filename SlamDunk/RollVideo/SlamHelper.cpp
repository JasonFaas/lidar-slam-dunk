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
		returnImg.at<uchar>(depthImage.at<uchar>(rowOfInterest, n), n) = 255;
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
		returnImg.at<uchar>((int)std::round(valueToSet), n) = 255;
	});

	return returnImg;
}

cv::Mat
SlamHelper::linesOnCommonFeatures(cv::Mat depthImage, cv::Mat overheadImage)
{
	frameTracker++;
	cv::Mat fullRepresentation = cv::Mat(cv::Size(1600, 800), CV_8UC3, cv::Scalar(0, 0, 0));
	int totalXOffset = 400;
	int totalYOffset = 100;

	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);
	
	int rowOfInterest = depthImage.rows / 2;
	const int featureLookAheadMax = 3;
	const int depthRangeAllowable = 6;
	const int minPointsInLine = 15;
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
				cv::Point* newStartPoint = new cv::Point(currentStartX,currentStartDepth);
				cv::Point* newEndPoint = new cv::Point(i, currentDepthRef);
				bool newFeature = true;
				for (DepthFeature* existingFeature : existingFeatures)
				{
					if (existingFeature->recentCloseToNewFeature(newStartPoint, newEndPoint, frameTracker))
					{
						newFeature = false;
						newFeatures.push_back(existingFeature);
						break;
					}
				}
				if (newFeature)
				{
					std::stringstream dispText;
					dispText << std::setfill('0') << std::setw(3) << featureNameIterator++;
					DepthFeature* currentFeature = new DepthFeature(dispText.str(), newStartPoint, newEndPoint, NULL, NULL, frameTracker);
					newFeatures.push_back(currentFeature);
					existingFeatures.push_back(currentFeature);
				}
			}
			currentStartX = i + 1;
			currentStartDepth = depthImage.at<uchar>(rowOfInterest, currentStartX);
			currentDepthRef = currentStartDepth;
		}
	}




	// highlight locations of features
	cv::Point* startPoint;
	cv::Point* endPoint;
	cv::Point currRobotPoint = cv::Point(totalXOffset + DEPTH_WIDTH / 2, totalYOffset);
	bool firstRobotLocation = true;
	for (DepthFeature* newFeature : newFeatures)
	{	
		std::tie(startPoint, endPoint) = newFeature->getRecentPoints();


		//TODO if new feature is existing and non-edge, attempt to update robot position
		if (newFeature->isOldFeature() && !newFeature->featureRecentOnEdge() && newFeature->getFeatureName() == "005")
		{
			// For PoC draw lines and show distances of all three lines
			// and also show all 3 angles
			// for RECENT
			int distanceMain = newFeature->twoPointDistance(startPoint, endPoint);
			int distanceLeft = newFeature->twoPointDistance(startPoint, &currRobotPoint);
			int distanceRight = newFeature->twoPointDistance(&currRobotPoint, endPoint);
			cv::putText(fullRepresentation, std::to_string(distanceMain), cv::Point(startPoint->x + totalXOffset, startPoint->y - 40 + totalYOffset), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(250, 150, 150));
			cv::putText(fullRepresentation, std::to_string(distanceLeft), cv::Point(startPoint->x + totalXOffset - 50, startPoint->y + 40 + totalYOffset), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 250, 150));
			cv::putText(fullRepresentation, std::to_string(distanceRight), cv::Point(endPoint->x + totalXOffset + 50, endPoint->y + 40 + totalYOffset), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 150, 250));

			// left
			cv::line(fullRepresentation, cv::Point(startPoint->x + totalXOffset, startPoint->y + totalYOffset), currRobotPoint, cv::Scalar(150, 250, 150), 3, 8, 0);
			// right
			cv::line(fullRepresentation, currRobotPoint, cv::Point(endPoint->x + totalXOffset, endPoint->y + totalYOffset), cv::Scalar(150, 150, 250), 3, 8, 0);

			// original circle
			cv::Point originalRobotLocation = newFeature->getOrigRobotLocationBasedOnRecentPoints();
			cv::circle(fullRepresentation, cv::Point(originalRobotLocation.x + totalXOffset, originalRobotLocation.y + totalYOffset), 4, cv::Scalar(100, 250, 255), -1);

			firstRobotLocation = false;
		}


		// Overhead Representaiton
		cv::line(overheadCopy, cv::Point(startPoint->x, startPoint->y), cv::Point(endPoint->x, endPoint->y), cv::Scalar(180), 3, 8, 0);
		cv::putText(overheadCopy, newFeature->getFeatureName(), cv::Point(startPoint->x, startPoint->y + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200));
		// Depth Representation
		cv::line(depthCopy, cv::Point(startPoint->x, rowOfInterest), cv::Point(endPoint->x, rowOfInterest), cv::Scalar(0), 3, 8, 0);
		cv::putText(depthCopy, newFeature->getFeatureName(), cv::Point(startPoint->x, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
		// Full Representation
		// TODO: This will be more complicated for actual feature tracking over multiple frames
		cv::line(fullRepresentation, cv::Point(startPoint->x + totalXOffset, startPoint->y + totalYOffset), cv::Point(endPoint->x + totalXOffset, endPoint->y + totalYOffset), cv::Scalar(180, 180, 180), 3, 8, 0);
		cv::circle(fullRepresentation, currRobotPoint, 4, cv::Scalar(100, 50, 255), -1);
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
