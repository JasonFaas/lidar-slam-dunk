#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>
#include "RollVideo.hpp"
#include <math.h>
#include <stdio.h>
#include <iomanip>
#include <numeric>
#include "../HelloOpenCV/StaticImageLogging.hpp"

RollVideo::RollVideo(const std::string& directory)
{
	resourceDir = directory;
}

void
RollVideo::rollVideo(std::string& videoFile)
{

}