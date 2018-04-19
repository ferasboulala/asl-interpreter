/* viewer.cpp
   test-node for the Viewer class
 */

#include "asl/Viewer.hpp"

using namespace std;
using namespace cv;

#define KINECT_V_RES 424
#define KINECT_H_RES 512

int main(int argc, char** argv){
	ros::init(argc, argv, "asl_viewer");
	ros::NodeHandle nh;
	Viewer * viewer = new Viewer(nh);

	Mat rgb, depth;
	vector<uint16_t> coords;
	namedWindow("rgb", WINDOW_NORMAL);
	namedWindow("depth", WINDOW_NORMAL);

	while(ros::ok()){
		if (viewer->getRGB(&rgb)){
			imshow("rgb", rgb);
		}
		if (viewer->getDepth(&depth)){
			depth = depth(Rect(0, 0, KINECT_H_RES, KINECT_V_RES));
			imshow("depth", depth);
		}
		if (viewer->getCoordinates(&coords)){
			ROS_INFO("x : %u, y: %u", coords[0], coords[1]);
		}
		waitKey(1);
		ros::spinOnce();
	}
	return 0;
}
