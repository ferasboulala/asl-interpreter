/* hand_tracker.cpp
   test-node for the HandTracker class
 */

#include "asl/HandTracker.hpp"

#define MAX_SHORT 65536

using namespace cv;


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "hand_tracker");
	ros::NodeHandle nh;

	HandTracker tracker(nh, PUBLISH, NO_DISP);
	tracker.startTracking();

	namedWindow("ASL Interpreter", CV_WINDOW_NORMAL);

	Mat wave = imread("hand_wave.png", CV_LOAD_IMAGE_UNCHANGED); // located in ~/catkin_ws/devel/lib/asl
	resize(wave, wave, Size(50,50), INTER_NEAREST);
	cvtColor(wave, wave, 1);
	wave.assignTo(wave, CV_16UC3);
	wave = wave * MAX_SHORT / 255.0;

	while (ros::ok())
	{
		tracker.spinner();

		Mat guiImage = tracker.returnDepth();
		guiImage = guiImage * MAX_SHORT / 4500.0;
		cvtColor(guiImage, guiImage, cv::COLOR_GRAY2RGB);
		tracker.draw(guiImage);

		if (tracker.returnCoordinates().size() == 0){ // Ask to wave hand
			int posX = 20, posY = 20;
			Rect zone = Rect(posX, posY, wave.cols, wave.rows);
			Mat waveROI = guiImage(zone);

			// The wave image has to be dominant over the depth image.
			for (int i = 0; i < wave.cols; i++){
				for (int j = 0; j < wave.rows; j++){
					if (wave.at<Vec3s>(i,j)[0] != 0 &&
							wave.at<Vec3s>(i,j)[1] != 0 &&
							wave.at<Vec3s>(i,j)[2] != 0){
						guiImage.at<Vec3s>(i + posX, j + posY)[0] = 0;
						guiImage.at<Vec3s>(i + posX, j + posY)[1] = 0;
						guiImage.at<Vec3s>(i + posX, j + posY)[2] = 0;
					}
				}
			}
			addWeighted(waveROI, 1, wave, 1, 0, waveROI);
			putText(guiImage, "Wave", Point(posX + wave.cols + 5, posY + wave.rows - 10), FONT_HERSHEY_SIMPLEX, 1, 
					Scalar(0, MAX_SHORT, MAX_SHORT), 2);
		}

		imshow("ASL Interpreter", guiImage);
		waitKey(1); 
		ros::spinOnce();
	}
	return 0;
}
