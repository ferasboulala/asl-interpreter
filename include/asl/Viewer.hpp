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

#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16MultiArray.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <sstream>

using namespace std;

typedef image_transport::SubscriberFilter ImageSubscriber;
typedef message_filters::sync_policies::ApproximateTime<
sensor_msgs::Image, sensor_msgs::Image> pol_;
typedef message_filters::Synchronizer<pol_> Sync;

/***********************************************
  Viewer
  This class is used to subscribe to an img topic
  and get the relevant image. It stores the images
  in its private attributes.
 ***********************************************/

class Viewer {
	public:
		/*
		   Constructor
		   @params: 1. ROS::NodeHandler
		 */
		Viewer(ros::NodeHandle nh);

		// Destructor
		~Viewer();

		/* Functions to access the rgb and depth images.
		   It makes a shallow copy to the input Mat images.
		   To get a deep copy, use cv::Mat.copyTo()
		   bool getRGB(cv::Mat* image);
Parameters: 1. Mat object that will hold the image.
Output: 1. true: received new image, false: didn't
receive new image.
		 */
		bool getRGB(cv::Mat* image);
		bool getDepth(cv::Mat* image);
		bool getCoordinates(vector<uint16_t> & coords);

	private:
		void callback(const sensor_msgs::ImageConstPtr& msgRGB,
				const sensor_msgs::ImageConstPtr& msgDepth);
		void handsCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg);

		image_transport::ImageTransport it_;

		ImageSubscriber sub_rgb, sub_depth;
		boost::shared_ptr<Sync> sync_;
		ros::Subscriber sub_;

		cv::Mat rgb_, depth_;
		std::vector<uint16_t> coords_;
		bool rgbOk, depthOk, coordsOk;
};

#endif
