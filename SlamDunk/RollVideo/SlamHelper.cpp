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

//cv::Mat
//SlamHelper::depthTo2DimAdjustedv1(cv::Mat& depthImage)
//{
//	cv::Mat returnImg = cv::Mat(255, depthImage.cols, CV_8UC1, cv::Scalar(0));
//
//	int rowOfInterest = depthImage.rows / 2;
//	Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
//		double depthAtROI = depthImage.at<uchar>(rowOfInterest, n);
//		double angleToCos = abs(90.0 - 55.0 - ((double)n) / 7.252);
//		double valueToSet = depthAtROI * cos(angleToCos * PI / 180.0);
//		//std::cout << "what:\t" << (int)depthAtROI << "\tmorewhat:\t" << (int)angleToCos << "\tmorewhat:\t" << (int)valueToSet << std::endl;
//		returnImg.at<uchar>((int)std::round(valueToSet), n) = 255;
//	});
//
//	return returnImg;
//}

cv::Mat
SlamHelper::depthTo2DimAdjusted(cv::Mat& depthImage) //v2 to fov
{
	cv::Mat returnImg = cv::Mat(255, depthImage.cols, CV_8UC1, cv::Scalar(0));

	int rowOfInterest = depthImage.rows / 2;
	Concurrency::parallel_for(0, depthImage.cols, 1, [&](int n) {
		double depthAtROI = depthImage.at<uchar>(rowOfInterest, n);
		double valueToSet = depthAtROI;
		//std::cout << "what:\t" << (int)depthAtROI << "\tmorewhat:\t" << (int)angleToCos << "\tmorewhat:\t" << (int)valueToSet << std::endl;
		//returnImg.at<uchar>(255 - (int)std::round(valueToSet), n) = 255;
		float fx = 365.456f / 1.8;
		//float fx = 150.456f;
		returnImg.at<uchar>((int)std::round(valueToSet), 50+(int)std::round(((double)n)*(valueToSet)/fx)) = 255;
		//returnImg.at<uchar>((int)std::round(valueToSet), 50+(int)std::round(((double)n)*(valueToSet+255/1.8)/fx)) = 255;
	});

	return returnImg;
}

cv::Mat
SlamHelper::linesOnCommonFeatures(cv::Mat& depthImage, cv::Mat& overheadImage)
{
	std::cout << "Frame:\t" << ++frameTracker << std::endl;

	// Place original robot location on totalRep
	if (frameTracker == 1)
		cv::circle(totalRep, cv::Point(TOTAL_X_OFFSET + SimpleStaticCalc::ROBOT_POS_X, TOTAL_Y_OFFSET + SimpleStaticCalc::ROBOT_POS_Y), 4, cv::Scalar(100, 50, 255), -1);

	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);

	linkExistingToNewFeatures(depthImage);
	drawLotsOfFeaturesV1(overheadCopy, depthCopy);

	//std::cout << "Feature Count:\t" << existingFeatures.size() << std::endl;
	cv::namedWindow("Overhead Extra");
	imshow("Overhead Extra", overheadCopy);
	cv::namedWindow("Depth Extra");
	imshow("Depth Extra", depthCopy);
	cv::namedWindow("Full Representation");
	imshow("Full Representation", totalRep);

	return depthCopy;
}

void
SlamHelper::linkExistingToNewFeatures(cv::Mat& depthImageRO)
{
	std::vector<DepthFeature> newFeatures = {};
	std::vector<DepthFeature> existingFeaturesInCurrent = {};

	const int featureLookAheadMax = SimpleStaticCalc::tuningfeatureLookAheadMax;
	const int depthRangeAllowable = SimpleStaticCalc::tuningdepthRangeAllowable;
	const int minPointsInLine = SimpleStaticCalc::tuningFeatureLengthMin;

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
						existingFeaturesInCurrent.push_back(existingFeature);
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

	std::cout << "Existing feature in current:\t" << existingFeaturesInCurrent.size() << std::endl;
	if (existingFeaturesInCurrent.size() == 0 && frameTracker > 1)
		std::cout << "\t!!!Existing feature in current count is ZERO!!!" << std::endl;

	cv::Point newStartPoint(-1, -1);
	cv::Point newEndPoint(-1, -1);
	cv::Point newF1StartPoint(-1, -1);
	cv::Point newF1EndPoint(-1, -1);
	for (DepthFeature& newFeature : newFeatures)
	{
		std::vector<int> newFeatureFrameOneStartIdeasX = {};
		std::vector<int> newFeatureFrameOneStartIdeasY = {};
		std::vector<int> newFeatureFrameOneEndIdeasX = {};
		std::vector<int> newFeatureFrameOneEndIdeasY = {};
		std::tie(newStartPoint, newEndPoint) = newFeature.getRecentPoints();

		for (DepthFeature& existingCurrentFeature : existingFeaturesInCurrent)
		{
			try {
				std::tie(newF1StartPoint, newF1EndPoint) = featureFrameOneGuess(newStartPoint, newEndPoint, existingCurrentFeature);
				newFeatureFrameOneStartIdeasX.push_back(newF1StartPoint.x);
				newFeatureFrameOneStartIdeasY.push_back(newF1StartPoint.y);
				newFeatureFrameOneEndIdeasX.push_back(newF1EndPoint.x);
				newFeatureFrameOneEndIdeasY.push_back(newF1EndPoint.y);
			}
			catch (std::invalid_argument e) {
				std::cout << "Invalid argument thrown attempting to guess at new feature Frame 1 point" << std::endl;
				std::cout << "Exception Details:\t" << e.what() << std::endl;
				continue;
			}
		}

		if (newFeatureFrameOneStartIdeasX.size() == 0)
		{
			if (frameTracker > 1)
				std::cout << "Error with feature on frame:\t" << newFeature.getFeatureName() << ":" << frameTracker << std::endl;
			continue;
		}

		// Set Frame One Location for features
		cv::Point finalStartGuess = SimpleStaticCalc::calculatePointsFromEstimations(newFeatureFrameOneStartIdeasX, newFeatureFrameOneStartIdeasY);
		cv::Point finalEndGuess = SimpleStaticCalc::calculatePointsFromEstimations(newFeatureFrameOneEndIdeasX, newFeatureFrameOneEndIdeasY);
		// TODO fix this inefficiency
		Concurrency::parallel_for_each(existingFeatures.begin(), existingFeatures.end(), [&](DepthFeature& df) {
			if (df.getFeatureName() == newFeature.getFeatureName())
				df.updateFrameOnePoints(finalStartGuess, finalEndGuess);
		});
	}
	whichExistingFeaturesDoNotHaveFrameOnePoints();
}

void
SlamHelper::drawLotsOfFeaturesV1(cv::Mat& overheadCopy, cv::Mat& depthCopy)
{
	// Get new robot position
	drawNewRobotLocation();
	
	Concurrency::parallel_for_each(existingFeatures.begin(), existingFeatures.end(), [&](DepthFeature & df) {

		try {
			cv::Point startPoint;
			cv::Point endPoint;
			std::tie(startPoint, endPoint) = df.getFrameOnePoints();
			cv::Point startPointRecent;
			cv::Point endPointRecent;
			std::tie(startPointRecent, endPointRecent) = df.getRecentPoints();

			if (df.isCurrentAndPrevious(frameTracker) || df.isBrandNew(frameTracker))
			{
				// Overhead Representaiton
				cv::line(overheadCopy, startPointRecent, endPointRecent, cv::Scalar(180), 3, 8, 0);
				cv::putText(overheadCopy, df.getFeatureName(), cv::Point(startPointRecent.x, startPointRecent.y + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200));
				// Depth Representation
				cv::line(depthCopy, cv::Point(startPoint.x, rowOfInterest), cv::Point(endPoint.x, rowOfInterest), cv::Scalar(0), 3, 8, 0);
				cv::putText(depthCopy, df.getFeatureName(), cv::Point(startPoint.x, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
			}
			if (df.isBrandNew(frameTracker))
			{
				// Full Representation
				int totalLineColor = 120 + frameTracker * 15 % (250 - 120);
				cv::line(totalRep, cv::Point(startPoint.x + TOTAL_X_OFFSET, startPoint.y + TOTAL_Y_OFFSET), cv::Point(endPoint.x + TOTAL_X_OFFSET, endPoint.y + TOTAL_Y_OFFSET), cv::Scalar(totalLineColor, totalLineColor, totalLineColor), 3, 8, 0);
			}
		}
		catch (std::invalid_argument e) {
			std::cout << "Invalid argument thrown accessing recent Frame One Points for display" << std::endl;	
		}
	});
}

std::tuple<cv::Point, cv::Point>
SlamHelper::featureFrameOneGuess(cv::Point& newStartPoint, cv::Point& newEndPoint, DepthFeature& existingCurrentFeature)
{
	cv::Point recentStartPoint(-1, -1);
	cv::Point recentEndPoint(-1, -1);
	cv::Point recentF1StartPoint(-1, -1);
	cv::Point recentF1EndPoint(-1, -1);
	cv::Point newF1StartPoint(-1, -1);
	cv::Point newF1EndPoint(-1, -1);
	double ideaStartAngle, ideaEndAngle = -1;

	std::tie(recentStartPoint, recentEndPoint) = existingCurrentFeature.getRecentPoints();
	std::tie(recentF1StartPoint, recentF1EndPoint) = existingCurrentFeature.getFrameOnePoints();

	// newStartPoint Info
	bool aboveStartEndLine = SimpleStaticCalc::aboveSlopeOfMainLine(recentStartPoint, recentEndPoint, newStartPoint);
	std::tie(ideaStartAngle, ideaEndAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(recentStartPoint, recentEndPoint, newStartPoint);
	newF1StartPoint = SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(recentF1StartPoint, recentF1EndPoint, ideaStartAngle, ideaEndAngle, aboveStartEndLine);

	// newEndPoint Info
	aboveStartEndLine = SimpleStaticCalc::aboveSlopeOfMainLine(recentStartPoint, recentEndPoint, newEndPoint);
	std::tie(ideaStartAngle, ideaEndAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(recentStartPoint, recentEndPoint, newEndPoint);
	newF1EndPoint = SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(recentF1StartPoint, recentF1EndPoint, ideaStartAngle, ideaEndAngle, aboveStartEndLine);

	return std::make_tuple(newF1StartPoint, newF1EndPoint);
}

//TODO DELETE THIS DEBUG METHOD
void
SlamHelper::whichExistingFeaturesDoNotHaveFrameOnePoints()
{
	Concurrency::parallel_for_each(existingFeatures.begin(), existingFeatures.end(), [&](DepthFeature& dp) {
		if (!dp.canAccessFrameOnePoints())
			std::cout << "!!! " << dp.getFeatureName() << " Does not have frame ONe points" << std::endl;
	});
}

void
SlamHelper::drawNewRobotLocation()
{
	std::vector<int> previousRobotGuessX = {};
	std::vector<int> previousRobotGuessY = {};
	std::mutex values_mutex;
	Concurrency::parallel_for_each(existingFeatures.begin(), existingFeatures.end(), [&](DepthFeature & df) {
		if (df.isCurrentAndPrevious(frameTracker))
		{
			try {
				cv::Point robotLocationNew = df.getNewRobotLocation();

				values_mutex.lock();
				previousRobotGuessX.push_back(robotLocationNew.x);
				previousRobotGuessY.push_back(robotLocationNew.y);
				values_mutex.unlock();
			}
			catch (std::invalid_argument e) {
				std::cout << "error getting new robot position:\t" << e.what() << std::endl;
			}
		}
	});
	if (previousRobotGuessX.size() > 0)
	{
		cv::Point finalGuess = SimpleStaticCalc::calculatePointsFromEstimations(previousRobotGuessX, previousRobotGuessY);
		cv::Point robotEstimation = cv::Point(finalGuess.x + TOTAL_X_OFFSET, finalGuess.y + TOTAL_Y_OFFSET);

		int h = (int)((frameTracker * 15) % 180);
		int s = 200;
		int v = 200;
		cv::Mat rgb;
		cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
		cv::cvtColor(hsv, rgb, CV_HSV2BGR);
		cv::Scalar newScalar(rgb.data[0], rgb.data[1], rgb.data[2]);

		cv::circle(totalRep, robotEstimation, 4, newScalar, -1);
	}
}