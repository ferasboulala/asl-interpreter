#include "asl/Preprocessor.hpp"

Preprocessor::Preprocessor(){

}

Preprocessor::~Preprocessor(){

}

// Evaluates the distance by averaging a neighborhood of pixels.
float Preprocessor::distance(Mat depth){
	int depthPixel;
	for (int i = -3; i < 4; i++){
		for (int j = -3; j < 4; j++){
			depthPixel = depth.at<short>(i + ceil(depth.rows/2) , j + ceil(depth.cols/2));
			if (abs(depthPixel - 800.0) < 300)
				return depthPixel;
		}
	}
	return USUAL_DEPTH;
}

// Applies a mask to the laplacian of the depth image
int Preprocessor::depthMask(Mat depth, Mat laplacian, int dist){
	if (depth.cols != laplacian.cols || depth.rows != laplacian.rows){
		cout << "[ERROR] : depthMask() images are not the same size" << endl;
		return -1;
	}

	for (int i = 0; i < depth.rows; i++){
		for (int j = 0; j< depth.cols; j++){
			if (abs(depth.at<ushort>(i,j) - dist) > 150 || laplacian.at<ushort>(i,j) > 200){
				laplacian.at<ushort>(i,j) = 0;
			}
		}
	}
	return 0;
}

// Applies a mask to the laplacian of the rgb image
int Preprocessor::depthMaskRGB(Mat depth, Mat laplacian, int dist){
	if (depth.cols != laplacian.cols || depth.rows != laplacian.rows){
		cout << "[ERROR] : depthMask() images are not the same size" << endl;
		return -1;
	}

	for (int i = 0; i < depth.rows; i++){
		for (int j = 0; j< depth.cols; j++){
			if (abs(depth.at<ushort>(i,j) - dist) > 130){
				laplacian.at<char>(i,j) = 0;
			}
		}
	}
	return 0;
}

// If the registered function does not work, you can use this approximation to center the 
// depth and rgb image after registration.
void Preprocessor::maskResize(Mat depth, Mat* mask){
	resize(*mask, *mask, Size(floor(mask->cols * 0.91), floor(mask->rows * 0.91)), INTER_NEAREST);
	int top = depth.rows - mask->rows - 1;
	int left = ceil((depth.cols - mask->cols)/2.0);
	int right = floor((depth.cols - mask->cols)/2.0);
	copyMakeBorder(*mask, *mask, top, 1, left, right, BORDER_CONSTANT, 0);
}

// Applies a mask. Useful if you want to keep the mask but slower since you
// have to go though the image twice in total (generate and apply a mask).
void Preprocessor::applyMask(Mat img, Mat mask){
	for (int i = 0; i < img.rows; i++){
		for (int j = 0; j< img.cols; j++){
			if (!mask.at<char>(i,j) || img.at<ushort>(i,j) > 300){
				img.at<ushort>(i,j) = 0;
			}
		}
	}
}

// Process function to call for the depth classification (CNN)
Mat Preprocessor::process(Mat depth, vector<uint16_t> coords){
	uint16_t x, y;
	int roiSize, dist, x_, y_, col, row;

	// Nearest hand is the first in the vector
	x = coords[0];
	y = coords[1];

	dist = depth.at<ushort>(y, x);
	if (dist < 500)
		dist = 500;

	// The region of interest is bigger as the hand gets closer.
	// If the ROI passes the depth image size, the ROI stays at the
	// border of the depth image.
	roiSize = SIZE_RATIO * 750 / dist;

	x_ = x - roiSize/2;
	y_ = y - roiSize/2;

	col = roiSize;
	row = roiSize;

	if (x_ < 0){
		x_ = 0;
	}
	if (col + x_ > depth.cols){
		col = depth.cols - x_;
	}

	if (y_ < 0){
		y_ = 0;
	}
	if (row + y_ > depth.rows){
		row = depth.rows - y_;
	}

	depth = depth(Rect(x_, y_, col, row));
	resize(depth, depth, Size(120,120), INTER_NEAREST); // All images are 120 x 120

	Mat laplacian;
	Laplacian(depth, laplacian, CV_16UC1, 3, BORDER_DEFAULT);

	depthMask(depth, laplacian, dist);

	laplacian = laplacian * 256 / 750.0;

	return laplacian;
}

// Process function to call for the rgb classification (SVM)
Mat Preprocessor::rgbProcess(Mat depth, Mat rgb, vector<uint16_t> coords){
	uint16_t x, y;
	int roiSize, dist, x_, y_, col, row;

	// Nearest hand is the first in the vector
	x = coords[0];
	y = coords[1];

	dist = depth.at<ushort>(y, x);
	if (dist < 500)
		dist = 500;

	// The region of interest is bigger as the hand gets closer.
	// If the ROI passes the depth image size, the ROI stays at the
	// border of the depth image.
	roiSize = SIZE_RATIO * 750 / dist;

	x_ = x - roiSize/2;
	y_ = y - roiSize/2;

	col = roiSize;
	row = roiSize;

	if (x_ < 0){
		x_ = 0;
	}
	if (col + x_ > depth.cols){
		col = depth.cols - x_;
	}

	if (y_ < 0){
		y_ = 0;
	}
	if (row + y_ > depth.rows){
		row = depth.rows - y_;
	}

	depth = depth(Rect(x_, y_, col, row));
	rgb = rgb(Rect(x_, y_, col, row));
	resize(depth, depth, Size(120,120), INTER_NEAREST);
	resize(rgb, rgb, Size(120,120), INTER_NEAREST);

	Mat laplacian;
	cvtColor(rgb, rgb, cv::COLOR_RGB2GRAY);
	Laplacian(rgb, laplacian, CV_8UC1, 3, BORDER_DEFAULT);

	depthMaskRGB(depth, rgb, dist);

	return rgb; // change for laplacian and in depthMaskRGB()
}