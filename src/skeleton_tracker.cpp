/* skeleton_tracker.cpp
   test-node for the SkeletonTracker class
 */

#include "asl/SkeletonTracker.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "skeleton_tracker");

	ros::NodeHandle nh;

	SkeletonTracker* tracker = new SkeletonTracker(&nh, PUBLISH, NO_DISP);

	while (ros::ok())
	{
		tracker->spinner();
	}

	delete tracker;

	return 0;
}
