/* SkeletonTracker.cpp
   Implementation of the SkeletonTracker.hpp class
 */

#include "asl/SkeletonTracker.hpp"

SkeletonTracker::SkeletonTracker(ros::NodeHandle & nh, bool cast, bool disp)
	: Sensor(nh, cast, false){
		ros::NodeHandle pnh("~");
		if (!pnh.getParam("tf_prefix", tf_prefix_))
		{
			ROS_FATAL("tf_prefix not found on Param Server! Maybe you should add it to your launch file!");
			ros::shutdown();
			return;
		}

		if (!pnh.getParam("relative_frame", relative_frame_))
		{
			ROS_FATAL("relative_frame not found on Param Server! Maybe you should add it to your launch file!");
			ros::shutdown();
			return;
		}

		if (!pnh.getParam("camera_frame", camera_frame_))
		{
			ROS_FATAL("camera_frame not found on Parameter Server! Maybe you should add it to your launch file!");
			ros::shutdown();
			return;
		}

		niteRc_ = nite::NiTE::initialize();
		if (niteRc_ != nite::STATUS_OK){
			ROS_FATAL("NiTE initialization failed\n");
			return;
		}

		niteRc_ = userTracker_.create();
		if (niteRc_ != nite::STATUS_OK){
			ROS_FATAL("Could not create user tracker");
			ros::shutdown();
			nite::NiTE::shutdown();
			return;
		}

		if (broadcast_){

		}
		pointCloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("asl/camera/point_cloud", 5);
		pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("asl/users", 1);

		for (int i = 0; i < 3; i++){
			coordinates_.layout.dim.push_back(std_msgs::MultiArrayDimension());
			coordinates_.layout.dim[i].size = 1;
			coordinates_.layout.dim[i].stride = 1;
		}

		coordinates_.layout.dim[0].label = "userID";
		coordinates_.layout.dim[1].label = "x";
		coordinates_.layout.dim[2].label = "y";

		display_ = disp;

		if (display_){
			cv::namedWindow("skeleton-depth", cv::WINDOW_NORMAL);
			cv::namedWindow("skeleton-rgb", cv::WINDOW_NORMAL);
		}

		ROS_INFO("User tracker INITIALIZED");
	}

SkeletonTracker::~SkeletonTracker(){
	nite::NiTE::shutdown();
}

void SkeletonTracker::spinner(){
	this->getRgb();
	this->getDepth();

	if (pointCloudPub_.getNumSubscribers() > 0)
		this->castPC();

	this->skeletonTrack();
	if (display_)
		this->display();

	rate_->sleep();
}

void SkeletonTracker::castPC(){
	float centerX, centerY;
	unsigned depthStep = 1, depthSkip = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZRGB>());

	if (depthStream_.readFrame(&depthFrame_) == openni::STATUS_OK){
		unsigned color_step, color_skip;
		openni::DeviceInfo info = dev_.getDeviceInfo();
		const char* uri = info.getUri();
		std::string stringa(uri);
		openni2_wrapper::OpenNI2Device dev(stringa);

		cloud_msg->header.stamp = 0;
		cloud_msg->width = depthFrame_.getWidth();
		cloud_msg->height = depthFrame_.getHeight();
		centerX = (cloud_msg->width >> 1) - 0.5f;
		centerY = (cloud_msg->height >> 1) - 0.5f;
		cloud_msg->is_dense = false;
		cloud_msg->points.resize((unsigned long)(cloud_msg->height * cloud_msg->width));
		color_step = 3 * msg_->width / cloud_msg->width;
		color_skip = 3 * (msg_->height / cloud_msg->height - 1) * msg_->width;

		const uint8_t* rgb_buffer = &msg_->data[0];
		const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame_.getData();

		float bad_point = std::numeric_limits<float>::quiet_NaN();

		float constant = 0.001 / dev.getDepthFocalLength(depthFrame_.getHeight());

		cloud_msg->header.frame_id = relative_frame_;

		int color_idx = 0, depth_idx = 0;
		pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_msg->begin();

		for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip){
			for (int u = 0; u < (int)cloud_msg->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter){
				pcl::PointXYZRGB& pt = *pt_iter;

				if (pDepth[depth_idx] == 0 || pDepth[depth_idx] > 10000){
					pt.x = pt.y = pt.z = bad_point;
					continue;
				}

				pt.x = -(u - centerX) * pDepth[depth_idx] * constant;
				pt.y = -(v - centerY) * pDepth[depth_idx] * constant;
				pt.z = pDepth[depth_idx] * 0.001;
				RGBValue color;
				color.Red = rgb_buffer[color_idx];
				color.Green = rgb_buffer[color_idx + 1];
				color.Blue = rgb_buffer[color_idx + 2];
				color.Alpha = 0;
				pt.rgb = color.float_value;
			}
		}

		sensor_msgs::PointCloud2 pc;
		pcl::toROSMsg(*cloud_msg, pc);
		pc.header.stamp = ros::Time::now();
		pointCloudPub_.publish(pc);
	}
}

void SkeletonTracker::publishSkeletons(){
	int size = users_.size();

	if (size == 0)
		return;

	coordinates_.data.clear();

	float x, y;
	for (int i = 0; i < size; ++i){
		userTracker_.convertJointCoordinatesToDepth(users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
				users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y,
				users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z,
				&x, &y);

		coordinates_.data.push_back(uint16_t(users_[i].getId()));
		coordinates_.data.push_back(uint16_t(x));
		coordinates_.data.push_back(uint16_t(y));
	}

	pub_.publish(coordinates_);
}

void SkeletonTracker::updateUserState(const nite::UserData& user, unsigned long long ts){
	if (user.isNew() && !g_visibleUsers_[user.getId()])
		USER_MESSAGE("New user")
	else if (user.isVisible() && !g_visibleUsers_[user.getId()])
		USER_MESSAGE("Visible user")
	else if (user.isLost())
		USER_MESSAGE("Lost a user")
	else if (user.isVisible() && !g_visibleUsers_[user.getId()])
		USER_MESSAGE("User out of scene")

			g_visibleUsers_[user.getId()] = user.isVisible();

	if (g_visibleUsers_[user.getId()] != user.getSkeleton().getState())
	{
		switch (g_skeletonStates_[user.getId()] = user.getSkeleton().getState())
		{
			case nite::SKELETON_NONE:
				USER_MESSAGE("Stopped tracking.")
					break;
			case nite::SKELETON_CALIBRATING:
				USER_MESSAGE("Calibrating...")
					break;
			case nite::SKELETON_TRACKED:
				//USER_MESSAGE("Tracking!")
				break;
			case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
			case nite::SKELETON_CALIBRATION_ERROR_HANDS:
			case nite::SKELETON_CALIBRATION_ERROR_LEGS:
			case nite::SKELETON_CALIBRATION_ERROR_HEAD:
			case nite::SKELETON_CALIBRATION_ERROR_TORSO:
				USER_MESSAGE("Calibration Failed... :-|")
					break;
		}
	}
}

void SkeletonTracker::skeletonTrack(){
	niteRc_ = userTracker_.readFrame(&userTrackerFrame_);
	if(niteRc_ != nite::STATUS_OK){
		ROS_ERROR("Could not get the next frame");
		return;
	}

	const nite::Array<nite::UserData>& users = userTrackerFrame_.getUsers();
	users_.clear();
	for (int i = 0; i < users.getSize(); ++i){
		const nite::UserData& user = users[i];
		updateUserState(user, userTrackerFrame_.getTimestamp());
		if (user.isNew()){
			userTracker_.startSkeletonTracking(user.getId());
		}

		else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED){
			std::map<std::string, nite::SkeletonJoint> named_joints;
			named_joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
			named_joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
			named_joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
			named_joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
			named_joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
			named_joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
			named_joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
			named_joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
			named_joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
			named_joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
			named_joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
			named_joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
			named_joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
			named_joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
			named_joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

			users_.push_back(user);
		}
	}
	publishSkeletons();
}

void SkeletonTracker::drawJoints(){
	for (int i = 0; i < users_.size(); i++){
		int radius = 10;
		//for (uint j = nite::JOINT_HEAD; j < nite::JOINT_RIGHT_FOOT; j++){ // how to iterate through enum ?
		float x, y, z;
		x = users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x;
		y = users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y;
		z = users_[i].getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().z;
		userTracker_.convertJointCoordinatesToDepth(x, y, z, &x, &y);
		cv::circle(depth_, cv::Point(int(x), int(y)), radius, cv::Scalar(4499));
		//}
	}
}

void SkeletonTracker::display(){
	drawJoints();
	cv::imshow("skeleton-depth", depth_ * 256 * 256 / 4500);
	cv::imshow("skeleton-rgb", rgb_);
	cv::waitKey(1);
}
