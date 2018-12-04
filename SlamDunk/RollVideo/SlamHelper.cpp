#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "SlamHelper.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "DepthFeature.hpp"
#include <numeric>
#include "SimpleStaticCalc.hpp"

SlamHelper::SlamHelper()
{
}

cv::Mat
SlamHelper::blurGoodDataOverBad(cv::Mat& depthImage) 
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
	cv::Mat dilationDst, blackPixelsThreshold, medianBlackPixels, tempDepthImg, maskInv;
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
SlamHelper::depthTo2D(cv::Mat& depthImage)
{
	cv::Mat returnImg = cv::Mat(255, depthImage.cols, CV_8UC1, cv::Scalar(0));

	int rowOfInterest = depthImage.rows / 2;
	Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
		returnImg.at<uchar>(depthImage.at<uchar>(rowOfInterest, n), n) = 255;
	});

	return returnImg;
}

cv::Mat
SlamHelper::depthTo2DimAdjusted(cv::Mat& depthImage)
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
SlamHelper::linesOnCommonFeatures(cv::Mat& depthImage, cv::Mat& overheadImage)
{
	frameTracker++;

	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);

	std::vector<DepthFeature> newFeatures = realizeNewFeatureAndLinkExisting(depthImage);
	drawLotsOfFeaturesV1(newFeatures, overheadCopy, depthCopy);

	std::cout << "Feature Count:\t" << existingFeatures.size() << std::endl;
	cv::namedWindow("Overhead Extra");
	imshow("Overhead Extra", overheadCopy);
	cv::namedWindow("Depth Extra");
	imshow("Depth Extra", depthCopy);
	cv::namedWindow("Full Representation");
	imshow("Full Representation", totalRep);

	return depthCopy;
}

std::vector<DepthFeature>
SlamHelper::realizeNewFeatureAndLinkExisting(cv::Mat& depthImageRO)
{
	std::vector<DepthFeature> newFeatures = {};

	const int featureLookAheadMax = 3;
	const int depthRangeAllowable = 6;
	const int minPointsInLine = 15;
	//std::vector<std::tuple<cv::Point, cv::Point>> traditionalFeatures = {};

	int currentStartX = 0;
	int currentStartDepth = depthImageRO.at<uchar>(rowOfInterest, currentStartX);
	int currentDepthRef = currentStartDepth;
	for (int i = 0; i < depthImageRO.cols; i++)
	{
		bool continueLine = false;
		for (int m = 0; m < featureLookAheadMax; m++)
		{
			int nextDepth = depthImageRO.at<uchar>(rowOfInterest, i + 1 + m);
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

		if (!continueLine || i + 2 + featureLookAheadMax > depthImageRO.cols)
		{
			if (i + 1 - currentStartX >= minPointsInLine)
			{
				cv::Point newStartPoint(currentStartX, currentStartDepth);
				cv::Point newEndPoint(i, currentDepthRef);
				bool newFeature = true;
				for (DepthFeature& existingFeature : existingFeatures)
				{
					if (existingFeature.recentCloseToNewFeature(newStartPoint, newEndPoint, frameTracker))
					{
						newFeature = false;
						existingFeature.addNewFeatureFrame(newStartPoint, newEndPoint, frameTracker);
						break;
					}
				}
				if (newFeature)
				{
					std::stringstream dispText;
					dispText << std::setfill('0') << std::setw(3) << featureNameIterator++;
					DepthFeature currentFeature(dispText.str(), newStartPoint, newEndPoint, frameTracker);
					newFeatures.push_back(currentFeature);
					existingFeatures.push_back(currentFeature);
				}
			}
			currentStartX = i + 1;
			currentStartDepth = depthImageRO.at<uchar>(rowOfInterest, currentStartX);
			currentDepthRef = currentStartDepth;
		}
	}

	return newFeatures;
}

void
SlamHelper::drawLotsOfFeaturesV1(std::vector<DepthFeature>& newFeatures, cv::Mat& overheadCopy, cv::Mat& depthCopy)
{
	cv::Point startPoint;
	cv::Point endPoint;
	cv::Point currRobotPoint = cv::Point(TOTAL_X_OFFSET + SimpleStaticCalc::DEPTH_WIDTH / 2, TOTAL_Y_OFFSET);

	std::vector<int> previousRobotGuessX = {};
	std::vector<int> previousRobotGuessY = {};

	std::cout << "Frame:\t" << frameTracker << std::endl;

	// Get new robot position
	for (DepthFeature& existingFeature : existingFeatures)
	{
		// TODO if feature is displayed on current frame and previous frame
		if (existingFeature.isCurrentAndMostRecentFrame(frameTracker))
		{
			// TODO then get angle between current location of feature and current location of robot
			// TODO Tuesday
			cv::Point robotLocationNew = existingFeature.getNewRobotLocationRelativeToPreviousLocation();
			std::cout << "\tLocation to point to:\t" << existingFeature.getFeatureName() << std::endl;
		}

	}


	for (DepthFeature& newFeature : newFeatures)
	{
		std::tie(startPoint, endPoint) = newFeature.getRecentPoints();

		// TODO: Reenable all this for actual desired result
		//if (newFeature.isRecentFeature())
		//{
			//// original robot position
			//cv::Point originalRobotLocation = newFeature.getOrigRobotLocationBasedOnRecentPoints();
			//previousRobotGuessX.push_back(originalRobotLocation.x);
			//previousRobotGuessY.push_back(originalRobotLocation.y);
		//}


		// Overhead Representaiton
		cv::line(overheadCopy, cv::Point(startPoint.x, startPoint.y), cv::Point(endPoint.x, endPoint.y), cv::Scalar(180), 3, 8, 0);
		cv::putText(overheadCopy, newFeature.getFeatureName(), cv::Point(startPoint.x, startPoint.y + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200));
		// Depth Representation
		cv::line(depthCopy, cv::Point(startPoint.x, rowOfInterest), cv::Point(endPoint.x, rowOfInterest), cv::Scalar(0), 3, 8, 0);
		cv::putText(depthCopy, newFeature.getFeatureName(), cv::Point(startPoint.x, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
		// Full Representation
		int totalLineColor = 120 + frameTracker * 15 % (250 - 120);
		cv::line(totalRep, cv::Point(startPoint.x + TOTAL_X_OFFSET, startPoint.y + TOTAL_Y_OFFSET), cv::Point(endPoint.x + TOTAL_X_OFFSET, endPoint.y + TOTAL_Y_OFFSET), cv::Scalar(totalLineColor, totalLineColor, totalLineColor), 3, 8, 0);
		cv::circle(totalRep, currRobotPoint, 4, cv::Scalar(100, 50, 255), -1);
	}

	if (previousRobotGuessX.size() > 0)
	{
		int robotGuessX = 0;
		int robotGuessY = 0;
		int robotGuesses = previousRobotGuessX.size();

		Concurrency::parallel_for_each(previousRobotGuessX.begin(), previousRobotGuessX.end(), [&](int n) {
			robotGuessX += n;
		});
		Concurrency::parallel_for_each(previousRobotGuessY.begin(), previousRobotGuessY.end(), [&](int n) {
			robotGuessY += n;
		});
		int yFinal = (int)std::round(robotGuessY / robotGuesses + TOTAL_Y_OFFSET);

		cv::Point robotEstimation = cv::Point((int)std::round(robotGuessX) / robotGuesses + TOTAL_X_OFFSET, yFinal);
		cv::circle(totalRep, robotEstimation, 4, cv::Scalar(250, 250, 100), -1);
	}
	else {
		std::cout << "Maybe Later" << std::endl;
	}
}