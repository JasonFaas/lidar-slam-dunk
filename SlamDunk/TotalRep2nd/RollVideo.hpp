#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include <ppl.h>

class RollVideo
{
	public:
		RollVideo(const std::string& directory);
		void rollVideo(std::string& videoFile);

	private:
		std::string resourceDir;
		
};
