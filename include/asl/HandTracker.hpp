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

#ifndef HANDTRACKER_HPP_
#define HANDTRACKER_HPP_

#include "asl/Sensor.hpp"
#include "std_msgs/Bool.h"

#define HT_RATIO 40000

/***********************************************
  HandTracker
  This class derives from @ref::Sensor class. It 
  publishes depth and rgb maps aswell as hand depth
  coordinates through NiTE.
 ***********************************************/

class HandTracker: public Sensor {
	public:
		/*
		   Constructor
		   @params: 1. ROS::NodeHandler 2. Broadcast: true: publish rgb and depth topics
		   3. Display: display rgb and depth images.
		 */
		HandTracker(ros::NodeHandle & nh, bool cast, bool disp);

		// Destructor
		~HandTracker();

		// Spinner function to call within a loop. ros::spinOnce() has to
		// be called inside the loop too.
		void spinner();

		// Call this function before HandTracker::spinner()
		void startTracking();

		// Call this function after HandTracker::startTracking()
		void stopTracking();

		// Attribute access methods
		std::vector<uint16_t> returnCoordinates();
		bool clicked();
		void draw(cv::Mat & im);

	private:
		void display();
		void handTrack();

		nite::HandTracker handTracker_;
		nite::HandTrackerFrameRef handTrackerFrame_;
		nite::Status niteRc_;

		std_msgs::UInt16MultiArray coordinates_;
		std_msgs::Bool clicked_;
		std::vector<uint16_t> hands_;

		ros::Publisher pub_, click_;

		int dist_, count_[2];
};

#endif
