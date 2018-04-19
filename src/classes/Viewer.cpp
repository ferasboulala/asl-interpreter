/* Viewer.cpp
   Implementation of the Viewer.hpp class
 */

#include "asl/Viewer.hpp"

Viewer::Viewer(ros::NodeHandle nh):
	it_(nh),
	sub_rgb(it_, "asl/camera/rgb", 1),
	sub_depth(it_, "asl/camera/depth", 1){
		sync_.reset(new Sync(pol_(10), sub_rgb, sub_depth));
		rgbOk = false;
		depthOk = false;
		coordsOk = false;
		sync_->registerCallback(boost::bind(&Viewer::callback, this, _1, _2));

		sub_ = nh.subscribe("asl/hands/coords", 1, &Viewer::handsCallback, this);

		ROS_INFO("Viewer is instanced with the current node");
	}

Viewer::~Viewer(){

}

bool Viewer::getRGB(cv::Mat* image){
	if (rgbOk){
		*image = rgb_;
		rgbOk = false;
		return true;
	}
	return false;
}

bool Viewer::getDepth(cv::Mat* image){
	if (depthOk){
		*image = depth_;
		depthOk = false;
		return true;
	}
	return false;
}

bool Viewer::getCoordinates(vector<uint16_t> & coords){
	if (coordsOk && coords_.size() > 0){
		coords.clear();
		for (vector<uint16_t>::const_iterator it = coords_.begin();
				it != coords_.end(); it++){
			coords.push_back(*it);
		}
		coordsOk = false;
		return true;
	}
	return false;
}

void Viewer::callback(const sensor_msgs::ImageConstPtr& msgRGB,
		const sensor_msgs::ImageConstPtr& msgDepth){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msgRGB);
		rgb_ = cv_ptr->image;
		rgbOk = true;

		cv_ptr = cv_bridge::toCvCopy(msgDepth);
		depth_ = cv_ptr->image;
		depthOk = true;
		return;
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_brdige exception: %s", e.what());
		return;
	}
}

void Viewer::handsCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg){
	if (msg->data.size() > 0){
		coordsOk = true;
		coords_.clear();
		for (vector<uint16_t>::const_iterator it = msg->data.begin();
				it != msg->data.end(); it++){
			coords_.push_back(*it);
		}
	}
}
