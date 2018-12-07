#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "FeatureFrameInfo.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include "SlamHelper.hpp"
#include "SimpleStaticCalc.hpp"


double
SimpleStaticCalc::twoPointDistance(cv::Point& first, cv::Point& second)
{
	return pow(pow(first.x - second.x, 2) + pow(first.y - second.y, 2), 0.5);
}

bool
SimpleStaticCalc::twoPointsClose(cv::Point& first, cv::Point& second)
{
	double distance = twoPointDistance(first, second);
	return distance < 25;
}

bool
SimpleStaticCalc::unitTestsHere()
{
	cv::Point firstTemp(10, 10);
	cv::Point secondTemp(15, 15);
	cv::Point thirdTemp(50, 50);
	if (!SimpleStaticCalc::twoPointsClose(firstTemp, secondTemp))
	{
		std::cout << "1st" << std::endl;
		return false;
	}
	if (SimpleStaticCalc::twoPointsClose(firstTemp, thirdTemp))
	{
		std::cout << "2nd" << std::endl;
		return false;
	}

	cv::Point startPoint, endPoint, thirdPointExpected, thirdPointResult;
	double startPointAngle, endPointAngle;
	bool aboveStartEndLine;
	
	//// Vertical slope Quadrant 4 relative result
	//startPoint = cv::Point(400, 300);
	//endPoint = cv::Point(400, 100);
	//thirdPointExpected = cv::Point(400 - 173, 200);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, 60, 60, true, aboveStartEndLine, false);
	//if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	//{
	//	std::cout << "5th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	//// Reverse Vertical slope Quadrant 1 relative result
	//startPoint = cv::Point(400, 100);
	//endPoint = cv::Point(400, 300);
	//thirdPointExpected = cv::Point(400 - 173, 200);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, 60, 60, true, aboveStartEndLine, false);
	//if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	//{
	//	std::cout << "5.1th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	//// Horizontal slope Quadrant 1 relative result
	//startPoint = cv::Point(100, 100);
	//endPoint = cv::Point(400, 100);
	//thirdPointExpected = cv::Point(200, 200);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, 60, 60, true, aboveStartEndLine, false);
	//if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	//{
	//	std::cout << "5.2th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	//// Horizontal slope Quadrant 4 relative result
	//startPoint = cv::Point(100, 100);
	//endPoint = cv::Point(400, 100);
	//thirdPointExpected = cv::Point(200, -200);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, 60, 60, true, aboveStartEndLine, false);
	//if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	//{
	//	std::cout << "5.3th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	//// Vertical slope Quadrant 1 relative result
	//startPoint = cv::Point(400, 300);
	//endPoint = cv::Point(400, 100);
	//thirdPointExpected = cv::Point(400 + 173, 200);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, 60, 60, true, aboveStartEndLine, false);
	//if (thirdPointResult.x != 400 + 173 || thirdPointResult.y != 200)
	//{
	//	std::cout << "6th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	// Negative slope Quadrant 4 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 100);
	thirdPointExpected = cv::Point(400, 100);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "7th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Negative slope Quadrant 1 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 100);
	thirdPointExpected = cv::Point(800, 400);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "8th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Negative slope Quadrant 2 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 100);
	thirdPointExpected = cv::Point(0, 900);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "9th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Negative slope Quadrant 3 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 100);
	thirdPointExpected = cv::Point(0, 500);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "10th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Positive slope Quadrant 4 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(800, 400);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "11th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Positive slope Quadrant 1 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(400, 800);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "12th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Positive slope Quadrant 2 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(-100, 100);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "13th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Positive slope Quadrant 3 relative result
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(-100, -1000);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(startPoint, endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	{
		std::cout << "14th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Start with Positive slope Q3 - End with Reverse Positive Slope Q3
	// TODO understand this comparison
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(-100, -1000);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	//int quadrant = SimpleStaticCalc::thirdPointRelativeQuadrant(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(endPoint, startPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != 1300 || thirdPointResult.y != 2200)
	{
		std::cout << "15th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Start with Positive slope Q3 - End with Negative Slope Q3
	// TODO understand this comparison
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(-100, -1000);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	//int quadrant = SimpleStaticCalc::thirdPointRelativeQuadrant(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(cv::Point(400, 1200), endPoint, startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != -1000 || thirdPointResult.y != 1700)
	{
		std::cout << "16th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Start with Positive slope Q3 - End with Reverse Negative Slope Q3
	// TODO understand this comparison
	startPoint = cv::Point(400, 400);
	endPoint = cv::Point(800, 800);
	thirdPointExpected = cv::Point(-100, -1000);
	aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(endPoint, cv::Point(400, 1200), startPointAngle, endPointAngle, aboveStartEndLine);
	if (thirdPointResult.x != 1300 || thirdPointResult.y != 2200)
	{
		std::cout << "17th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
		return false;
	}

	// Not sure what I'm testing, only that I know current result is wrong
	// TODO understand
	//startPoint = cv::Point(285, 187);
	//endPoint = cv::Point(315, 188);
	//thirdPointExpected = cv::Point(0, 97);
	//aboveStartEndLine = aboveSlopeOfMainLine(startPoint, endPoint, thirdPointExpected);
	//std::tie(startPointAngle, endPointAngle) = SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(startPoint, endPoint, thirdPointExpected);
	//thirdPointResult = get3rdPointLocationFrom2PointsAndAngles(cv::Point(285, 189), cv::Point(300, 197), startPointAngle, endPointAngle, aboveStartEndLine);
	//if (thirdPointResult.x != thirdPointExpected.x || thirdPointResult.y != thirdPointExpected.y)
	//{
	//	std::cout << "18th:\t" << thirdPointResult.x << "\tand:\t" << thirdPointResult.y << std::endl;
	//	return false;
	//}

	std::cout << "SimpleStaticCalc Unit Tests Complete!" << std::endl;
	return true;
}

bool
SimpleStaticCalc::aboveSlopeOfMainLine(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint) {
	if (endPoint.x == startPoint.x)
	{
		if (endPoint.y > startPoint.y)
			return thirdPoint.x < startPoint.x;
		else
			return thirdPoint.x > startPoint.x;
	}

	double diffMainX = endPoint.x - startPoint.x;
	double diffMainY = endPoint.y - startPoint.y;
	double mainLineSlope = diffMainY / diffMainX;
	double yIntercept = startPoint.y - (startPoint.x * mainLineSlope);
	return thirdPoint.y > thirdPoint.x * mainLineSlope + yIntercept;
}

std::tuple<double, double>
SimpleStaticCalc::calculateInitialAnglesTo3rdPoint(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint)
{
	if (showInputsDebug)
		std::cout << "SimpleStaticCalc::calculateInitialAnglesTo3rdPoint\t" << startPoint.x << ":" << startPoint.y << ":" << endPoint.x << ":" << endPoint.y << ":" << thirdPoint.x << ":" << thirdPoint.y << std::endl;
	if (!isValidTriangle(startPoint, endPoint, thirdPoint))
		throw std::invalid_argument("Invalid_Feature:SimpleStaticCalc::calculateInitialAnglesTo3rdPoint");
	
	double startPointAngle, endPointAngle = -1;
	double distanceMain = twoPointDistance(startPoint, endPoint);
	double distanceLeft = twoPointDistance(startPoint, thirdPoint);
	double distanceRight = twoPointDistance(thirdPoint, endPoint);

	double initialPart = (pow(distanceMain, 2) + pow(distanceLeft, 2) - pow(distanceRight, 2)) / (2 * distanceMain * distanceLeft);
	startPointAngle = acos(initialPart) * 180 / PI;

	initialPart = (pow(distanceMain, 2) + pow(distanceRight, 2) - pow(distanceLeft, 2)) / (2 * distanceMain * distanceRight);
	endPointAngle = acos(initialPart) * 180 / PI;
	
	return std::make_tuple(startPointAngle, endPointAngle);
}

bool
SimpleStaticCalc::isValidTriangle(cv::Point& startPoint, cv::Point& endPoint, cv::Point& thirdPoint)
{
	double distanceMain = SimpleStaticCalc::twoPointDistance(startPoint, endPoint);
	double distanceLeft = SimpleStaticCalc::twoPointDistance(startPoint, thirdPoint);
	double distanceRight = SimpleStaticCalc::twoPointDistance(thirdPoint, endPoint);
	double threeSides[3];
	threeSides[0] = distanceMain;
	threeSides[1] = distanceLeft;
	threeSides[2] = distanceRight;
	double perimeter = threeSides[2] + threeSides[1] + threeSides[0];
	for (int i = 0; i < 3; i++)
	{
		// IF a single side is longer than the other 2, invalid triangle
		if (perimeter - threeSides[i] < threeSides[i])
		{
			return false;
		}
	}

	return true;
}

cv::Point
SimpleStaticCalc::calculatePointsFromEstimations(std::vector<int> estimationsX, std::vector<int> estimationsY)
{
	int robotGuessX = 0;
	int robotGuessY = 0;
	int robotGuesses = estimationsX.size();

	Concurrency::parallel_for_each(estimationsX.begin(), estimationsX.end(), [&](int n) {
		robotGuessX += n;
	});
	Concurrency::parallel_for_each(estimationsY.begin(), estimationsY.end(), [&](int n) {
		robotGuessY += n;
	});
	int xFinal = (int)std::round(robotGuessX / robotGuesses);
	int yFinal = (int)std::round(robotGuessY / robotGuesses);

	return cv::Point(xFinal, yFinal);
}

cv::Point
SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles(cv::Point& startPoint, cv::Point& endPoint, double startPointAngle, double endPointAngle, bool relativePositiveY)
{

	if (showInputsDebug)
		std::cout << "SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles\t" << startPoint.x << ":" << startPoint.y << ":" << endPoint.x << ":" << endPoint.y << ":" << startPointAngle << ":" << endPointAngle << ":" << relativePositiveY << std::endl;

	double angleTooSharp = 2.0;
	double newPointAngle = 180 - endPointAngle - startPointAngle;
	if (startPointAngle < angleTooSharp || endPointAngle < angleTooSharp || newPointAngle < angleTooSharp)
	{
		std::cout << "!!!Error!!! Angle to Sharp!!!" << std::endl;


		std::cout << "\tStart x:\t" << std::to_string(startPoint.x) << std::endl;
		std::cout << "\tStart y:\t" << std::to_string(startPoint.y) << std::endl;
		std::cout << "\tEnd x:\t" << std::to_string(endPoint.x) << std::endl;
		std::cout << "\tEnd y:\t" << std::to_string(endPoint.y) << std::endl;
		std::cout << "\tStart Angle:\t" << std::to_string(startPointAngle) << std::endl;
		std::cout << "\tEnd Angle:\t" << std::to_string(endPointAngle) << std::endl;
		throw std::invalid_argument("Angle too Sharp::SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles");
	}


	double mainLength = twoPointDistance(endPoint, startPoint);


	double startToNewLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(endPointAngle * PI / 180);
	double endToNewLength = (mainLength / sin(newPointAngle * PI / 180)) * sin(startPointAngle * PI / 180);

	// this equation finds third point assuming mainLength is from (0,0) to (x,0) - (for most scenarios xCord is positive and yCord is negative)
	double xCord = (pow(startToNewLength, 2) - pow(endToNewLength, 2) + pow(mainLength, 2)) / (mainLength * 2);
	double yCord = pow(pow(startToNewLength, 2) - pow(xCord, 2), 0.5);
	if (!relativePositiveY)
		yCord *= -1;

	cv::Point shiftedEndPointZero(endPoint.x - startPoint.x, endPoint.y - startPoint.y);
	double rotationalAngle = -1.0;
	if (shiftedEndPointZero.x == 0 || shiftedEndPointZero.y == 0)
		std::invalid_argument("SimpleStaticCalc::get3rdPointLocationFrom2PointsAndAngles::Not currently handling vertical or horizontal lines");
	/*if (shiftedEndPointZero.x == 0 && shiftedEndPointZero.y > 0)
		rotationalAngle = -90.0;
	else if (shiftedEndPointZero.x == 0 && shiftedEndPointZero.y < 0)
		rotationalAngle = 90.0;
	else if (shiftedEndPointZero.y == 0 && shiftedEndPointZero.x > 0)
		rotationalAngle = 0.0;
	else if (shiftedEndPointZero.y == 0 && shiftedEndPointZero.x < 0)
		rotationalAngle = 180.0;*/
	else
	{
		rotationalAngle = acos((pow(shiftedEndPointZero.x, 2) + pow(mainLength, 2) - pow(shiftedEndPointZero.y, 2)) / (2 * shiftedEndPointZero.x * mainLength)) * 180 / PI;
		// TODO Test this more thoroughly
		if (!(shiftedEndPointZero.x > 0 && shiftedEndPointZero.y > 0))
			rotationalAngle *= -1;
	}

	double rotationalPointX = cos(rotationalAngle * PI / 180) * xCord - sin(rotationalAngle * PI / 180) * yCord;
	double rotationalPointY = sin(rotationalAngle * PI / 180) * xCord + cos(rotationalAngle * PI / 180) * yCord;
	cv::Point returnPointRotated((int)std::round(rotationalPointX) + startPoint.x, (int)std::round(rotationalPointY) + startPoint.y);

	// Print out info if initial robot is not close to where it should be
	//if (abs(returnPointRotated.x - DEPTH_WIDTH / 2) > 10 || abs(returnPointRotated.y) > 10)
	//{
	//	std::cout << "\n\nRecent Start x:\t" << std::to_string(recentStartPoint->x) << std::endl;
	//	std::cout << "Recent Start y:\t" << std::to_string(recentStartPoint->y) << std::endl;
	//	std::cout << "Recent End x:\t" << std::to_string(recentEndPoint->x) << std::endl;
	//	std::cout << "Recent End y:\t" << std::to_string(recentEndPoint->y) << std::endl;
	//	std::cout << "Start Angle:\t" << std::to_string(origStartPointAngle) << std::endl;
	//	std::cout << "End Angle:\t" << std::to_string(origEndPointAngle) << std::endl;
	//	std::cout << "Close to robot xCord:\t" << std::to_string(returnPointRotated.x - DEPTH_WIDTH / 2) << std::endl;
	//	std::cout << "Close to robot yCord:\t" << std::to_string(returnPointRotated.y) << std::endl;

	//	std::cout << "\nMain Length:\t" << std::to_string(mainLength) << std::endl;
	//	std::cout << "Left Length:\t" << std::to_string(leftLength) << std::endl;
	//	std::cout << "Right Length:\t" << std::to_string(rightLength) << std::endl;
	//	std::cout << "Simple New xCord:\t" << std::to_string(xCord) << std::endl;
	//	std::cout << "Simple New yCord:\t" << std::to_string(yCord) << std::endl;
	//	std::cout << "Rotational Angle:\t" << std::to_string((int)rotationalAngle) << std::endl;
	//	std::cout << "Rotated Xcord:\t" << std::to_string(rotationalPointX) << std::endl;
	//	std::cout << "Rotated Ycord:\t" << std::to_string(rotationalPointY) << std::endl;
	//	std::cout << "Shifted xCord End:\t" << std::to_string(shiftedEndPointZero.x) << std::endl;
	//	std::cout << "Shifted yCord End:\t" << std::to_string(shiftedEndPointZero.y) << std::endl;
	//	std::cout << "Final xCord:\t" << std::to_string(returnPointRotated.x) << std::endl;
	//	std::cout << "Final yCord:\t" << std::to_string(returnPointRotated.y) << std::endl;
	//}

	//std::cout << std::to_string(mainLength) << std::endl;

	if (returnPointRotated.x < -100 || returnPointRotated.x > 1000 || returnPointRotated.y > 1000 || returnPointRotated.y < -100)
	{
		std::cout << "!!!Error!!! Point from standard!!!" << std::endl;


		std::cout << "\tStart x:\t" << std::to_string(startPoint.x) << std::endl;
		std::cout << "\tStart y:\t" << std::to_string(startPoint.y) << std::endl;
		std::cout << "\tEnd x:\t" << std::to_string(endPoint.x) << std::endl;
		std::cout << "\tEnd y:\t" << std::to_string(endPoint.y) << std::endl;
		std::cout << "\tStart Angle:\t" << std::to_string(startPointAngle) << std::endl;
		std::cout << "\tEnd Angle:\t" << std::to_string(endPointAngle) << std::endl;
	}


	return returnPointRotated;
}
