/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2018 (c) Feras Boulala
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met: none
 *********************************************************************/
#ifndef PREPROCESSOR_HPP_
#define PREPROCESSOR_HPP_

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

#define USUAL_DEPTH 700
#define RATIO 30.0
#define SIZE_RATIO 120

using namespace cv;
using namespace std;

/***********************************************
  Preprocessor
  This class takes a depth image with hand coordinates
  and returns a processed image for the classifier.
 ***********************************************/

class Preprocessor{
	public:
		// Constructor
		Preprocessor();

		//Destructor
		~Preprocessor();

		// Process functions for both depth and rgb classification
		Mat process(Mat depth, vector<uint16_t> coords);
		Mat rgbProcess(Mat depth, Mat rgb, vector<uint16_t> coords);

	private:
		// Functions called within the process() functions above
		float distance(Mat depth);
		int depthMask(Mat depth, Mat laplacian, int dist);
		int depthMaskRGB(Mat depth, Mat laplacian, int dist);
		void maskResize(Mat depth, Mat* mask);
		void applyMask(Mat img, Mat mask);	
};

#endif
