/* Sensor.cpp
   Implementation of the Sensor.hpp class
 */

#include "asl/Sensor.hpp"

Sensor::Sensor(ros::NodeHandle & nh, bool cast, bool disp):
	it_(nh), broadcast_(cast), display_(disp)
{
	nh_ = nh;

	if (openni::OpenNI::initialize() != openni::STATUS_OK){
		ROS_FATAL("OpenNI error");
		ros::shutdown();
		return;
	}

	if (dev_.open(openni::ANY_DEVICE) != openni::STATUS_OK){
		ROS_FATAL("Could not open the device");
		ros::shutdown();
		return;
	}

	if (depthStream_.create(dev_, openni::SENSOR_DEPTH) == openni::STATUS_OK){

		for (int i = 0; i < depthStream_.getSensorInfo().getSupportedVideoModes().getSize(); i++){
			ROS_INFO("Depth[%d]: XRes: %d, YRes: %d", i,
					depthStream_.getSensorInfo().getSupportedVideoModes()[i].getResolutionX(),
					depthStream_.getSensorInfo().getSupportedVideoModes()[i].getResolutionY());
		}

		depthMode_ = depthStream_.getSensorInfo().getSupportedVideoModes()[1];

		if (depthStream_.setVideoMode(depthMode_) != openni::STATUS_OK){
			depthMode_ = depthStream_.getVideoMode();
		}

		depthStream_.setMirroringEnabled(true);
	}
	else {
		ROS_FATAL("Depth stream could not be initialized");
		ros::shutdown();
		return;
	}

	if (rgbStream_.create(dev_, openni::SENSOR_COLOR) == openni::STATUS_OK){
		rgbMode_.setResolution(H_RES, V_RES);
		rgbMode_.setFps(FPS);
		rgbMode_.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

		if (rgbStream_.setVideoMode(rgbMode_) != openni::STATUS_OK){
			rgbMode_ = rgbStream_.getVideoMode();
		}

		if (dev_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			dev_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

		rgbStream_.setMirroringEnabled(true);
	}
	else {
		ROS_FATAL("RGB stream could not be initialized");
		ros::shutdown();
		return;
	}

	rgbStream_.start();
	depthStream_.start();

	if (broadcast_){
		rgbPub_ = it_.advertise("asl/camera/rgb", 1);
		depthPub_ = it_.advertise("asl/camera/depth", 1);
	}

	if (display_){
		cv::namedWindow("depth", cv::WINDOW_NORMAL);
		cv::namedWindow("rgb", cv::WINDOW_NORMAL);
	}

	rate_ = new ros::Rate(100); // Change this rate to publish less frequently (higher is faster)

	gotOne_ = false;
	ROS_INFO("Sensor INITIALIZED");
}

Sensor::~Sensor(){
}

void Sensor::spinner(){
	this->getRgb();
	this->getDepth();

	if (display_){
		this->display();
	}

	rate_->sleep();
}

int Sensor::getRgb(){
	if (rgbStream_.readFrame(&rgbFrame_) != openni::STATUS_OK){
		ROS_ERROR("Could not get rgb frame");
		return -1;
	}
	else {
		const cv::Mat rgbMat(rgbFrame_.getHeight(), rgbFrame_.getWidth(), 
				CV_8UC3, const_cast<void*>((rgbFrame_).getData()));

		rgb_ = rgbMat;
		cv::cvtColor(rgb_, rgb_, cv::COLOR_RGB2BGR);

		if (!rgbMat.empty()){
			msg_ = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgbMat).toImageMsg();
			msg_->header.stamp = ros::Time::now();
			if (broadcast_)
				castRGB();
		}
		else {
			ROS_ERROR("cv_bridge rgb is empty");
			return -1;
		}
	}
	return 0;
}

int Sensor::getDepth(){
	depthStream_.start();
	depthStream_.readFrame(&depthFrame_);

	if(depthFrame_.isValid()){
		openni::DepthPixel* pData = (openni::DepthPixel*)depthFrame_.getData();

		cv::Mat depthMat(depthStream_.getVideoMode().getResolutionY(),
				depthStream_.getVideoMode().getResolutionX(), CV_16UC1, pData);

		depth_ = depthMat(cv::Rect(0, 0, DEPTH_H_RES, DEPTH_V_RES));

		if (!depthMat.empty()){
			cv_bridge::CvImage msg;
			msg.header.stamp = ros::Time::now();
			msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			msg.image = depthMat;

			gotOne_ = true;
			if (broadcast_)
				castDepth(&msg);
		}
		else{
			ROS_ERROR("cv_bridge depth is empty");
			return -1;
		}
	}
	else {
		ROS_ERROR("Could not get depth frame");
		return -1;
	}
	return 0;
}

void Sensor::castRGB(){
	rgbPub_.publish(msg_);
}

void Sensor::castDepth(const cv_bridge::CvImage* msg){
	depthPub_.publish(msg->toImageMsg());
}

void Sensor::display(){
	cv::imshow("depth", depth_ * 256 * 256 / 4500);
	cv::imshow("rgb", rgb_);
	cv::waitKey(1);
}

bool Sensor::isOk(){
	return gotOne_;
}

cv::Mat Sensor::returnRGB(){
	cv::Mat copy;
	rgb_.copyTo(copy);
	return copy;
}

cv::Mat Sensor::returnDepth(){
	cv::Mat copy;
	depth_.copyTo(copy);
	return copy;
}
