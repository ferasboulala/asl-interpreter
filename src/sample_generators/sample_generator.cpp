/* sample_generator.cpp
   Generates all the samples for training
   It must receive the type of the images and the dataset
   that it is generating (i.e. rgb or depth and training or
   validation). ORDER: type dataset.

   If no parameter is given, both depth and rgb samples are saved.

   To generate either laplacian or raw rgb images, the
   Preprocessor.cpp src file has to be recompiled to return
   laplacian images. Alternatively, a simple python script can
   be made to compute the laplacians.
 */

// TODO: Implement arrow keys to save on either RGB or DEPTH directories instead of both.
// This is useful if one type is chosen as the most efficient or if one would like
// to expand only one type at a time.

#include "asl/HandTracker.hpp"
#include "asl/Preprocessor.hpp"
#include "asl/Viewer.hpp"

#include <sstream>
#include <stdio.h>
#include <ctype.h>

using namespace std;
using namespace cv;

enum img_type {RGB_TYPE = 0, DEPTH_TYPE = 1, ALL_TYPE = 2};
enum training_set {TRAINING_SET = 0, VALIDATION_SET = 1};
enum control_keys {UP, DOWN, LEFT, RIGHT};

int main(int argc, char **argv) {
	ros::init(argc, argv, "hand_tracker");

	int symbols[2][2][36] = {0};

	ostringstream dataset_name[2];
	ostringstream img_name[2];

	dataset_name[0] << "samples/training/";
	dataset_name[1] << "samples/validation/";
	img_name[0] << "rgb/raw/";
	img_name[1] << "depth/";

	for (int i = 0; i < 2; i++){ // i:= training/validation
		for (int j = 0; j < 2; j++){ // j := rgb/depth
			for (int k = 0; k < 26; k++){ // k := letter
				char let = char(k + 97);
				int n = 0;
				while(true){
					ostringstream filename;
					filename << dataset_name[i].str() << img_name[j].str() << let << '/' << n << ".jpg";

					if (!imread(filename.str(), CV_LOAD_IMAGE_UNCHANGED).data){
						break;
					}
					n++;
				}
				symbols[i][j][k] = n;
				ROS_INFO("Letter %c : %d", let, symbols[i][j][k]);
			}

			for (int k = 26; k < 36; k++){
				int n = 0;
				while(true){
					ostringstream filename;
					filename << dataset_name[i].str() << img_name[j].str() << k - 26 << '/' << n << ".jpg";
					if (!imread(filename.str(), CV_LOAD_IMAGE_UNCHANGED).data){
						break;
					}
					n++;
				}
				symbols[i][j][k] = n;
				ROS_INFO("Number %d : %d", k - 26, symbols[i][j][k]);
			}			
		}
	}

	ros::NodeHandle nh;

	HandTracker * tracker = new HandTracker(nh, NO_PUBLISH, DISP);
	tracker->startTracking();
	Preprocessor * preprocessor = new Preprocessor();

	namedWindow("Raw rgb image", WINDOW_NORMAL);
	namedWindow("Laplacian depth image");


	cout << "-------- ASL Sample Generator --------\n"
		<< "1. Wave at the camera to detect your hand.\n"
		<< "2. Put your fingers in the right position and press the letter on the keyboard.\n";

	int img_tp = ALL_TYPE;
	bool valid_rgb = false, valid_depth = false;
	bool valid_rgb_n = false, valid_depth_n = false;

	while (ros::ok())
	{
		tracker->spinner();

		if (tracker->isOk() && tracker->returnCoordinates().size() > 0){
			Mat raw_rgb, laplacian_depth;

			laplacian_depth = preprocessor->process(tracker->returnDepth(), tracker->returnCoordinates());
			imshow("Laplacian depth image", laplacian_depth * 256);

			raw_rgb = preprocessor->rgbProcess(tracker->returnDepth(), tracker->returnRGB(),
						tracker->returnCoordinates());
			imshow("Raw rgb image", raw_rgb);

			int key = waitKey(33);

			ostringstream filename;
			filename << "samples/";

			switch(img_tp){
				case RGB_TYPE: {
					filename << "rgb/raw/";
				}
				break;
				case DEPTH_TYPE : {
					filename << "depth/";
				}
				break;
			}

			switch(key){
				case LEFT : {
					img_tp = RGB_TYPE;
					ROS_INFO("Working on RGB only.");
				}
				break;
				case RIGHT : {
					img_tp = DEPTH_TYPE;
					ROS_INFO("Working on Depth only.");
				}
				break;
				case UP : {
					img_tp = ALL_TYPE;
					ROS_INFO("Working on ALL image types.");
				}
				default : {
					if (key <= 122 && key >= 97){	
						int idx = key - 97;  	
						char let = (char(key));

						if (img_tp == ALL_TYPE){ // BOTH
							if (valid_rgb) {
								filename << "validation/" << "rgb/raw/" << let << '/' << symbols[VALIDATION_SET][RGB_TYPE][idx] << ".jpg";
								symbols[VALIDATION_SET][RGB_TYPE][idx]++;
								valid_rgb = false;
							}
							else {
								if (!(symbols[TRAINING_SET][RGB_TYPE][idx] % 10)){
									valid_rgb = true;
								}
								filename << "training/" << "rgb/raw/" << let << '/' << symbols[TRAINING_SET][RGB_TYPE][idx] << ".jpg";
								symbols[TRAINING_SET][RGB_TYPE][idx]++;
							}

							cout << filename.str() << endl;

							imwrite(filename.str(), raw_rgb);
							filename.str(std::string());

							filename << "samples/";
							if (valid_depth) {
								filename << "validation/" << "depth/" << let << '/' << symbols[VALIDATION_SET][DEPTH_TYPE][idx] << ".jpg";
								symbols[VALIDATION_SET][DEPTH_TYPE][idx]++;
								valid_depth = false;
							}
							else {
								if (!(symbols[TRAINING_SET][DEPTH_TYPE][idx] % 10)){
									valid_depth = true;
								}
								filename << "training/" << "depth/" << let << '/' << symbols[TRAINING_SET][DEPTH_TYPE][idx] << ".jpg";
								symbols[TRAINING_SET][DEPTH_TYPE][idx]++;
							}
							
							cout << filename.str() << endl;
							imwrite(filename.str(), laplacian_depth);							

							cout << "[RGB] Saved letter " << let << " n." << symbols[VALIDATION_SET][RGB_TYPE][idx] << endl;
							cout << "[DEPTH] Saved letter "<< let << " n." << symbols[VALIDATION_SET][DEPTH_TYPE][idx] << endl; 

						}
						else { // EITHER RGB OR DEPTH
							filename << let << '/' << symbols[0][img_tp][idx] << ".jpg";
							if (img_tp == RGB_TYPE){
								imwrite(filename.str(), raw_rgb);
							}
							else {
								imwrite(filename.str(), laplacian_depth);
							}
							symbols[TRAINING_SET][img_tp][idx];
						}
					}

					else if (key >= 48 && key <= 57){
						int digit = int(key) - 48, idx = digit + 26;

						if (img_tp = ALL_TYPE){ // BOTH
							if (valid_rgb_n) {
								filename << "validation/" << "rgb/raw/" << digit << '/' << symbols[VALIDATION_SET][RGB_TYPE][idx] << ".jpg";
								symbols[VALIDATION_SET][RGB_TYPE][idx]++;
								valid_rgb_n = false;
							}
							else {
								if (!(symbols[TRAINING_SET][RGB_TYPE][idx] % 10)){
									valid_rgb_n = true;
								}
								filename << "training/" << "rgb/raw/" << digit << '/' << symbols[TRAINING_SET][RGB_TYPE][idx] << ".jpg";
								symbols[TRAINING_SET][RGB_TYPE][idx]++;
							}
							
							imwrite(filename.str(), raw_rgb);
							filename.str(std::string());

							filename << "samples/";
							if (valid_depth_n) {
								filename << "validation/" << "depth/" << digit << '/' << symbols[VALIDATION_SET][DEPTH_TYPE][idx] << ".jpg";
								symbols[VALIDATION_SET][DEPTH_TYPE][idx]++;
								valid_depth_n = false;
							}
							else {
								if (!(symbols[TRAINING_SET][DEPTH_TYPE][idx] % 10)){
									valid_depth_n = true;
								}
								filename << "training/" << "depth/" << digit << '/' << symbols[TRAINING_SET][DEPTH_TYPE][idx] << ".jpg";
								symbols[TRAINING_SET][DEPTH_TYPE][idx]++;
							}
							

							imwrite(filename.str(), laplacian_depth);

							cout << "[RGB] Saved number " << digit << " n." << symbols[TRAINING_SET][RGB_TYPE][idx] << endl;
							cout << "[DEPTH] Saved number "<< digit << " n." << symbols[TRAINING_SET][DEPTH_TYPE][idx] << endl; 
						}
						else { // EITHER RGB OR DEPTH
							filename << digit << '/' << symbols[0][img_tp][idx] << ".jpg";
							if (img_tp == RGB_TYPE){
								imwrite(filename.str(), raw_rgb);
							}
							else {
								imwrite(filename.str(), laplacian_depth);
							}
							symbols[TRAINING_SET][img_tp][idx];
						}
					}

					else {
						if (key != -1)
							ROS_INFO("This is not a letter.");
					}
				}
				break;
			}
		}
		ros::spinOnce();
	}

	delete tracker;
	delete preprocessor;

	return 0;
}
