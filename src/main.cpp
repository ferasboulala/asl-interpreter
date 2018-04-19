/* main
   This node instances the appropriate objects and communicates
   with the classification nodes with images. Callbacks happen
   when the classifiers return a prediction on the ASL symbol
   that was sent.

   This node defaults to rgb detection. To use depth, start
   it with the argument depth.
 */

#include "asl/HandTracker.hpp"
#include "asl/Preprocessor.hpp"
#include "std_msgs/String.h"

#define MAX_SHORT 65536
#define N_CONSECUTIVE 3
#define YELLOW Scalar(0, MAX_SHORT, MAX_SHORT)
#define GREEN Scalar(0, MAX_SHORT, 0)

using namespace cv;

enum{
	NUMBER_TYPE=false, LETTER_TYPE=true
};

enum{
	RGB_TYPE=false, DEPTH_TYPE=true
};

struct symbolHandler{
	bool publishToClassifier;
	ostringstream currentSymbol, lastWritten, sentence;
	bool type, img_type; // type is letter or number
						// img_type is depth or rgb
	int consecutive, n_count;
	bool write;
};					   

symbolHandler handler;

void callback(const std_msgs::String::ConstPtr& letterMsg){
	if (letterMsg->data.c_str() != 0){
		if (handler.currentSymbol.str() == letterMsg->data.c_str() &&
			handler.currentSymbol.str() != handler.lastWritten.str()){
			handler.consecutive++;
		}
		else {
			handler.consecutive = 0;
		}

		handler.currentSymbol.str(std::string());
		handler.currentSymbol << letterMsg->data.c_str();

		if (handler.consecutive == N_CONSECUTIVE){
			handler.n_count++;
			if ((handler.type == LETTER_TYPE) || (handler.type == NUMBER_TYPE && handler.n_count <= 4)){
				handler.sentence << handler.currentSymbol.str();
			}
			ROS_INFO("Got enough consecutive classifications");
		}
	}
	handler.publishToClassifier = true;
}

void send(Mat image, image_transport::Publisher& pub, ros::Publisher& type_pub){
	std_msgs::Bool msg_type;
	msg_type.data = handler.type;
	type_pub.publish(msg_type);

	cv_bridge::CvImage msg;

	if (handler.img_type){
		msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	}
	else {
		msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	}

	msg.image = image;
	msg.header.stamp = ros::Time::now();
	pub.publish(msg.toImageMsg());
}

void displaySymbol(Mat & im){
	int size = int(im.cols/6);
	putText(im, handler.currentSymbol.str(), Point(size*5,size), FONT_HERSHEY_SIMPLEX, 2, 
			YELLOW, 2);

	string str;
	if (handler.type == NUMBER_TYPE){
		str = "Nombre";
	}
	else {
		str = "Lettre";
	}

	putText(im, str, Point(size * 4.5, size / 3), FONT_HERSHEY_SIMPLEX, 1, 
			GREEN, 2);
	if (handler.write){
		putText(im, handler.sentence.str(), Point(size, im.rows/5), 
				FONT_HERSHEY_SIMPLEX, 1, YELLOW, 2);
		if (handler.type == NUMBER_TYPE){
			putText(im, "Annee de naissance:", Point(0, size/2), FONT_HERSHEY_SIMPLEX, 1, YELLOW, 2);
		}
		else {
			putText(im, "Votre nom est:", Point(0, size/2), FONT_HERSHEY_SIMPLEX, 1, YELLOW, 2);
		}
	}
}

// Function that returns the string representing the type of a Mat object
string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}

void initHandler(int argc, char * argv[]){
	handler.type = NUMBER_TYPE;
	handler.publishToClassifier = false;
	handler.consecutive = 0;
	handler.write = false;
	handler.sentence.str(std::string());
	handler.currentSymbol.str(std::string());
	handler.lastWritten.str(std::string());

	if (argc > 1 && argv[argc - 1] == string("depth")){
		ROS_INFO("Processing depth images");
		handler.img_type = DEPTH_TYPE;
	}
	else {
		ROS_INFO("Processing rgb images");
		handler.img_type = RGB_TYPE;
	}
}

void resetHandler(){
	handler.sentence.str(std::string());
	handler.n_count = 0;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "hand_tracker");
	initHandler(argc, argv);

	ros::NodeHandle nh;
	HandTracker tracker(nh, NO_PUBLISH, NO_DISP);
	Preprocessor preprocessor;

	image_transport::ImageTransport it(nh);
	ros::Subscriber letter_sub = nh.subscribe("asl/letter", 1, callback);
	ros::Publisher type_pub = nh.advertise<std_msgs::Bool>("asl/hands/type", 1);
	image_transport::Publisher pub = it.advertise("asl/hands/right", 1);

	namedWindow("ASL Interpreter", CV_WINDOW_NORMAL);

	Mat wave = imread("hand_wave.png", CV_LOAD_IMAGE_UNCHANGED); // located at ~/catkin_ws/devel/lib/asl
	resize(wave, wave, Size(50,50), INTER_NEAREST);
	cvtColor(wave, wave, 1);
	wave.assignTo(wave, CV_16UC3);
	wave = wave * MAX_SHORT / 255.0;

	tracker.startTracking();
	while (ros::ok())
	{
		tracker.spinner();
		Mat guiImage = tracker.returnDepth();
		guiImage = guiImage * MAX_SHORT / 4500.0;
		cvtColor(guiImage, guiImage, COLOR_GRAY2RGB);

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
			putText(guiImage, "Balayez", Point(posX + wave.cols + 5, posY + wave.rows - 10), FONT_HERSHEY_SIMPLEX, 1, 
					YELLOW, 2);

			handler.sentence.str(std::string());
		} 
		else {
			tracker.draw(guiImage);
			displaySymbol(guiImage);

			if (handler.publishToClassifier && tracker.isOk()){
				Mat forClassifier;
				if (handler.img_type){
					forClassifier = preprocessor.process(tracker.returnDepth(),
							tracker.returnCoordinates());
				}
				else {
					forClassifier = preprocessor.rgbProcess(tracker.returnDepth(), tracker.returnRGB(),
							tracker.returnCoordinates());
				}     	
				send(forClassifier, pub, type_pub);

				// if (tracker.clicked()){
				// 	handler.type = !handler.type;
				// 	handler.sentence.str(std::string());
				// }
				handler.publishToClassifier = false;
			}
		} 

		imshow("ASL Interpreter", guiImage);

		// Project display state machine
		int cmd = waitKey(5); 
		switch(cmd){
			case 114:{ // r := reset
				resetHandler();
				ROS_INFO("Cleared sentence.");
				break;
			} 
			
			case 97:{ // a := letters (alpha)
				handler.type = LETTER_TYPE;
				resetHandler();
				ROS_INFO("Classifying letters.");
				break;
			}
			
			case 100:{ // d := numbers (digits)
				handler.type = NUMBER_TYPE;
				resetHandler();
				ROS_INFO("Classifying numbers.");
				break;
			}
			
			case 119:{ // w := write (toggle)
				handler.write = !handler.write;
				resetHandler();
				ROS_INFO("Toggled write command.");
				break;
			}
		}
		ros::spinOnce();
	}
	return 0;
}
