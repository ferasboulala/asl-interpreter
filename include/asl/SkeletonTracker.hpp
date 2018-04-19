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

#ifndef SKELETONTRACKER_HPP_
#define SKELETONTRACKER_HPP_

#include "asl/Sensor.hpp"

/***********************************************
SkeletonTracker
This class derives from @Sensor. It publishes
depth and rgb maps aswell as all user depth joint
coordinates.
***********************************************/

class SkeletonTracker: public Sensor {
public:
	/*
	Constructor
	@params: 1. ROS::NodeHandler 2. Broadcast: true: publish rgb and depth topics
			 3. Display: display rgb and depth images.
	*/
	SkeletonTracker(ros::NodeHandle & nh, bool cast, bool show);

	// Destructor
	~SkeletonTracker();

	// Spinner function to call within a loop
	void spinner();

	cv::Mat returnRGB();
	cv::Mat returnDepth();

private:
	void castPC();
	void publishSkeletons();
	void updateUserState(const nite::UserData& user, unsigned long long ts);
	void skeletonTrack();
	void drawJoints();
	// This function displays the skeleton. It is called within spinner.
	// The code can be changed depending on your needs. If the spinner()
	// function is too restrictive and you wish to call display() whenever,
	// you can make it public and delete its call from the spinner().
	void display();
	
	bool g_visibleUsers_[MAX_SKELETONS] = {false};

 	nite::SkeletonState g_skeletonStates_[MAX_SKELETONS] = {nite::SKELETON_NONE};
	nite::UserTracker userTracker_;
	nite::UserTrackerFrameRef userTrackerFrame_;
	nite::Status niteRc_;

	ros::Publisher pointCloudPub_, pub_;
	tf::TransformBroadcaster tfCast_;
	std::string tf_prefix_, relative_frame_, camera_frame_;

	std_msgs::UInt16MultiArray coordinates_;
	std::vector<nite::UserData> users_;
};

#endif