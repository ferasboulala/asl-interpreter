/* sample_generator.cpp
   Generates all the samples for the smaller dataset. It
   picks one every 15 images. It was created to see the
   result of training the CNN with a lot less images.
Results: Better training accuracy at the expense of a
horrible validation accuracy. Conclusions: Overfitting.
 */

#include <sstream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
	for (int i = 0; i < 26; i++){
		char let = char(i + 97);
		int n = 0;

		while(true){
			ostringstream filename;
			filename << "samples/" << let << '/' << n << ".jpg";
			Mat img = imread(filename.str(), CV_LOAD_IMAGE_UNCHANGED);

			if (!img.data){
				break;
			}

			n++;
			if (n % 15 == 0){
				ostringstream s;
				s << "samples_small/" << let << '/' << n << ".jpg";
				imwrite(s.str(), img);
			}
		}
	}

	for (int i = 26; i < 36; i++){
		int n = 0;

		while(true){
			ostringstream filename;
			filename << "samples/" << i - 26 << '/' << n << ".jpg";
			Mat img = imread(filename.str(), CV_LOAD_IMAGE_UNCHANGED);

			if (!img.data){
				break;
			}

			n++;
			if (n % 15 == 0){
				ostringstream s;
				s << "samples_small/" << i - 26 << '/' << n << ".jpg";
				imwrite(s.str(), img);
			}
		}
	}
}
