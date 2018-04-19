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

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <kdl/frames.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt16MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Std libs
#include <iostream>
#include <fstream>
#include <string.h>
#include <strings.h>
#include <vector>

// Natural Interactions
#include "NiTE.h"
#include <openni2/OpenNI.h>
#include <openni2/OniCEnums.h>
#include <openni2_camera/openni2_device.h>
#include <openni2/OniCTypes.h>

// Constantss
#define MAX_HANDS 10
#define MAX_SKELETONS 10
#define H_RES 1920
#define V_RES 1080
#define DEPTH_V_RES 424
#define DEPTH_H_RES 512
#define FPS 30
#define USER_MESSAGE(msg) \
{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);} // macro

enum Types {
	NO_PUBLISH = false, PUBLISH = true
};

enum Disp {NO_DISP = false, DISP = true
};

typedef union
{
	struct{
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	float float_value;
	long long_value;
} RGBValue;

typedef std::map<std::string, nite::SkeletonJoint> JointMap;

/***********************************************
  Sensor
  This class connects to a valid openni device and 
  publishes the depth and rgb maps.

  It is an alternative to the openni_launch package
  from which other classes like @ref::SkeletonTracker
  can derive from.
 ***********************************************/

class Sensor{
	public:
		/*
		   Constructor
		   @params: 1. ROS::NodeHandler 
		   2. Broadcast: true: publish or not rgb and depth topics
		   3. Display: display or not rgb and depth images.
		 */
		Sensor(ros::NodeHandle & nh, bool cast, bool disp);

		//Destructor
		~Sensor();

		// Spinner function to call within a loop. ros::spinceOnce()
		// still needs to be called.
		void spinner();
		bool isOk();
		cv::Mat returnRGB();
		cv::Mat returnDepth();

	protected:
		virtual void display();
		int getRgb();
		int getDepth();
		void castRGB();
		void castDepth(const cv_bridge::CvImage* msg);

		ros::NodeHandle nh_;

		openni::Device dev_;
		openni::VideoStream rgbStream_, depthStream_;
		openni::VideoFrameRef rgbFrame_, depthFrame_;
		openni::VideoMode rgbMode_, depthMode_;

		image_transport::ImageTransport it_;
		image_transport::Publisher rgbPub_, depthPub_;

		sensor_msgs::ImagePtr msg_;
		ros::Rate* rate_;

		cv::Mat rgb_, depth_;

		bool broadcast_;
		bool display_;
		bool gotOne_;
};

#endif
