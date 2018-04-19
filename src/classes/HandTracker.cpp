/* HandTracker.cpp
   Implementation of the HandTracker.hpp class
 */

#include "asl/HandTracker.hpp"

HandTracker::HandTracker(ros::NodeHandle & nh, bool cast, bool disp)
	: Sensor(nh, cast, false){
		niteRc_ = nite::NiTE::initialize();
		if (niteRc_ != nite::STATUS_OK){
			ROS_FATAL("NiTE initialization failed\n");
			return;
		}

		niteRc_ = handTracker_.create();
		if (niteRc_ != nite::STATUS_OK)
		{
			ROS_FATAL("Could not create hand tracker\n");
			return;
		}

		pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("asl/hands/coords", 1);
		click_ = nh_.advertise<std_msgs::Bool>("asl/hands/click", 1);

		clicked_.data = false;

		coordinates_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		coordinates_.layout.dim[0].size = 1;
		coordinates_.layout.dim[0].stride = 1;
		coordinates_.layout.dim[0].label = "coord";

		display_ = disp;
		count_[0] = count_[1] = 0;

		if (display_){
			cv::namedWindow("hand-depth", cv::WINDOW_NORMAL);
			cv::namedWindow("hand-rgb", cv::WINDOW_NORMAL);
		}

		ROS_INFO("HandTracker INITIALIZED");
	}

HandTracker::~HandTracker(){
	nite::NiTE::shutdown();
}

void HandTracker::spinner(){
	this->getRgb();
	this->getDepth();
	this->handTrack();

	if (display_){
		this->display();
	}

	//rate_->sleep();
}

void HandTracker::startTracking(){
	handTracker_.startGestureDetection(nite::GESTURE_WAVE);
	handTracker_.startGestureDetection(nite::GESTURE_CLICK);
	ROS_INFO("Wave your hand to start the ASL interpreter !");
}

void HandTracker::stopTracking(){
	handTracker_.stopGestureDetection(nite::GESTURE_WAVE);
	handTracker_.stopGestureDetection(nite::GESTURE_CLICK);
	ROS_INFO("IDLE");
}

void HandTracker::handTrack(){
	niteRc_ = handTracker_.readFrame(&handTrackerFrame_);
	if (niteRc_ != nite::STATUS_OK){
		ROS_INFO("NiTE could not access the next frame\n");
	}

	const nite::Array<nite::GestureData>& gestures = handTrackerFrame_.getGestures();
	for (int i = 0; i < gestures.getSize(); i++){
		if (gestures[i].isComplete()){
			if (gestures[i].getType() == nite::GESTURE_WAVE){
				nite::HandId newId;
				handTracker_.startHandTracking(gestures[i].getCurrentPosition(), &newId);
			}
			else {
				if (hands_.size() > 0){
					clicked_.data = true;
					click_.publish(clicked_);
					ROS_INFO("CLICKED");
				}
			}
		}
	}

	const nite::Array<nite::HandData>& hands = handTrackerFrame_.getHands();
	if (hands.getSize() > 0){
		for (int i = 0; i < 1; i++){
			count_[i] = count_[i + 1];
		}
		count_[1] = hands_.size();
		hands_.clear();
		float x, y;
		std::vector<uint16_t> temp;
		for (int i = 0; i < hands.getSize(); ++i){
			const nite::HandData& hand = hands[i];
			if (hand.isTracking()){
				handTracker_.convertHandCoordinatesToDepth(hand.getPosition().x,
						hand.getPosition().y, hand.getPosition().z, &x, &y);
				temp.push_back(uint16_t(x));
				temp.push_back(uint16_t(y));
			}
		}

		coordinates_.data.clear();
		coordinates_.data.insert(coordinates_.data.end(), temp.begin(),
				temp.end());

		pub_.publish(coordinates_);
		hands_ = temp;

		// Finding the nearest hand and placing it as the first element in the vector
		if (hands_.size() > 0 && count_[0] == count_[1]){
			int min = 0;
			for (int i = 0; i < hands_.size() - 1; i += 2){
				if (depth_.at<short>(hands_[min + 1], hands_[min]) >  
						depth_.at<short>(hands_[i + 1], hands_[i]) &&
						depth_.at<short>(hands_[i + 1], hands_[i]) != 0) {
					min = i;
				}
			}
			int x = hands_[0], y = hands_[1];
			hands_[0] = hands_[min];
			hands_[1] = hands_[min + 1];
			hands_[min] = x;
			hands_[min + 1] = y;
		}
	}
}

void HandTracker::draw(cv::Mat & im){
	if (hands_.size() > 0 && count_[0] == count_[1]){
		int min = 0;
		try{
			dist_ = 500;
			for (int i = 0; i < hands_.size() - 1; i += 2){
				int color = 0;
				int x = hands_[i];
				int y = hands_[i+1];
				int current = depth_.at<short>(y, x);
				int radius;

				if (current != 0){dist_ = current;}

				radius = int(HT_RATIO / dist_); // 40 pixels per meter

				if (i == min){
					color = 65536;
				}

				cv::line(im, cv::Point(x,0), cv::Point(x,im.rows-1), cv::Scalar(0,color,0));
				cv::line(im, cv::Point(0,y), cv::Point(im.cols-1,y), cv::Scalar(0,color,0));
				cv::rectangle(im, cv::Point(x-radius,y-radius), cv::Point(x+radius,y+radius), cv::Scalar(0,color,0));
			}
		}
		catch (cv::Exception& e){
			ROS_ERROR("Could not draw on hand");
		}
	}
}

void HandTracker::display(){
	cv::imshow("hand-depth", depth_ * 256 * 256 / 4500);
	cv::imshow("hand-rgb", rgb_);
	cv::waitKey(1);
}

std::vector<uint16_t> HandTracker::returnCoordinates(){
	return hands_;
}

bool HandTracker::clicked(){
	if (clicked_.data){
		clicked_.data = false;
		return true;
	}
	return false;
}
