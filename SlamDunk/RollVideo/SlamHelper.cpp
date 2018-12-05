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
	std::cout << "Frame:\t" << ++frameTracker << std::endl;

	cv::Mat depthCopy;
	cv::Mat overheadCopy;
	depthImage.copyTo(depthCopy);
	overheadImage.copyTo(overheadCopy);

	std::vector<DepthFeature> newFeatures = realizeNewFeatureAndLinkExisting(depthImage);
	drawLotsOfFeaturesV1(newFeatures, overheadCopy, depthCopy);

	//std::cout << "Feature Count:\t" << existingFeatures.size() << std::endl;
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
	// TODO May not need to return newFeatures variable as existingFeatures now has everything
	// TODO Wednesday Dec-5

	std::vector<DepthFeature> newFeatures = {};
	std::vector<DepthFeature> existingFeaturesInCurrent = {};

	const int featureLookAheadMax = 3;
	const int depthRangeAllowable = 6;
	const int minPointsInLine = 15;

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

	// TODONE for each new feature - should have an 'location based on frame 1' - 'frame1StartPoint' & 'frame1EndPoint'
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

		// Set Frame One Location for new Feature
		cv::Point finalStartGuess = SimpleStaticCalc::calculatePointsFromEstimations(newFeatureFrameOneStartIdeasX, newFeatureFrameOneStartIdeasY);
		cv::Point finalEndGuess = SimpleStaticCalc::calculatePointsFromEstimations(newFeatureFrameOneEndIdeasX, newFeatureFrameOneEndIdeasY);
		
		for (DepthFeature& existingFeature : existingFeatures)
		{
			if (existingFeature.getFeatureName() == newFeature.getFeatureName())
			{
				existingFeature.updateFrameOnePoints(finalStartGuess, finalEndGuess);
				break;
			}
		}
		newFeature.updateFrameOnePoints(finalStartGuess, finalEndGuess);
	}
	whichExistingFeaturesDoNotHaveFrameOnePoints(newFeatures);

	return newFeatures;
}

void
SlamHelper::drawLotsOfFeaturesV1(std::vector<DepthFeature>& newFeatures, cv::Mat& overheadCopy, cv::Mat& depthCopy)
{

	cv::Point startPoint;
	cv::Point endPoint;
	cv::Point currRobotPoint = cv::Point(TOTAL_X_OFFSET + SimpleStaticCalc::ROBOT_POS_X, TOTAL_Y_OFFSET + SimpleStaticCalc::ROBOT_POS_Y);

	std::vector<int> previousRobotGuessX = {};
	std::vector<int> previousRobotGuessY = {};

	// Get new robot position
	for (DepthFeature& existingFeature : existingFeatures)
	{
		if (existingFeature.isCurrentAndPrevious(frameTracker))
		{
			// TODO May want to save robotLocationNew (and other screen elements) so they can be quickly referenced as needed - other elements could easily be saved in FeatureFrameInfo
			cv::Point robotLocationNew = existingFeature.getNewRobotLocationRelativeToPreviousLocation();
			//std::cout << "\tLocation to point to:\t" << robotLocationNew.x << ":" << robotLocationNew.y << std::endl;

			previousRobotGuessX.push_back(robotLocationNew.x);
			previousRobotGuessY.push_back(robotLocationNew.y);
		}
		else if (existingFeature.isBrandNew(frameTracker))
		{
			// TODO delete this else if
			//std::cout << "\tBrand New Feature:\t" << existingFeature.getFeatureName() << std::endl;
		}

	}


	for (DepthFeature& newFeature : newFeatures)
	{
		//std::tie(startPoint, endPoint) = newFeature.getRecentPoints();
		// TODO Investigate replacing above with below
		try {
			std::tie(startPoint, endPoint) = newFeature.getFrameOnePoints();
		}
		catch (std::invalid_argument e) {
			std::cout << "Invalid argument thrown accessing recent Frame One Points for display" << std::endl;
			continue;
		}

		// Overhead Representaiton
		cv::line(overheadCopy, cv::Point(startPoint.x, startPoint.y), cv::Point(endPoint.x, endPoint.y), cv::Scalar(180), 3, 8, 0);
		cv::putText(overheadCopy, newFeature.getFeatureName(), cv::Point(startPoint.x, startPoint.y + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(200));
		// Depth Representation
		cv::line(depthCopy, cv::Point(startPoint.x, rowOfInterest), cv::Point(endPoint.x, rowOfInterest), cv::Scalar(0), 3, 8, 0);
		cv::putText(depthCopy, newFeature.getFeatureName(), cv::Point(startPoint.x, rowOfInterest + 40), CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar(0));
		// Full Representation
		int totalLineColor = 120 + frameTracker * 15 % (250 - 120);
		cv::line(totalRep, cv::Point(startPoint.x + TOTAL_X_OFFSET, startPoint.y + TOTAL_Y_OFFSET), cv::Point(endPoint.x + TOTAL_X_OFFSET, endPoint.y + TOTAL_Y_OFFSET), cv::Scalar(totalLineColor, totalLineColor, totalLineColor), 3, 8, 0);
	}
	cv::circle(totalRep, currRobotPoint, 4, cv::Scalar(100, 50, 255), -1);

	if (previousRobotGuessX.size() > 0)
	{
		cv::Point finalGuess = SimpleStaticCalc::calculatePointsFromEstimations(previousRobotGuessX, previousRobotGuessY);
		cv::Point robotEstimation = cv::Point(finalGuess.x + TOTAL_X_OFFSET, finalGuess.y + TOTAL_Y_OFFSET);
		cv::circle(totalRep, robotEstimation, 4, cv::Scalar(250, 250, 100), -1);
	}
	else {
		std::cout << "Maybe Later" << std::endl;
	}
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
	std::tie(ideaStartAngle, ideaEndAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(recentStartPoint, recentEndPoint, newStartPoint);
	newF1StartPoint = SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(recentF1StartPoint, recentF1EndPoint, ideaStartAngle, ideaEndAngle);

	// newEndPoint Info
	std::tie(ideaStartAngle, ideaEndAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(recentStartPoint, recentEndPoint, newEndPoint);
	newF1EndPoint = SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(recentF1StartPoint, recentF1EndPoint, ideaStartAngle, ideaEndAngle);

	if (newF1StartPoint.y > 1000 || newF1EndPoint.y > 1000)
	{
		std::cout << std::to_string(ideaStartAngle) << ":" << std::to_string(ideaEndAngle) << ":" << std::to_string(recentF1StartPoint.x) << ":" << std::to_string(recentF1StartPoint.y) << ":" << std::to_string(recentF1EndPoint.x) << ":" << std::to_string(recentF1EndPoint.y) << ":" << std::endl;
	}

	return std::make_tuple(newF1StartPoint, newF1EndPoint);
}

//TODO DELETE THIS DEBUG METHOD
void
SlamHelper::whichExistingFeaturesDoNotHaveFrameOnePoints(std::vector<DepthFeature> newFeatures)
{
	for (DepthFeature& existingFeature : existingFeatures)
	{
		if (!existingFeature.canAccessFrameOnePoints())
		{
			std::cout << "!!! " << existingFeature.getFeatureName() << " Does not have frame ONe points" << std::endl;
		}
	}
}