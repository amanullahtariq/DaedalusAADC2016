
#include <stdlib.h>   

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Mat frame, frame_roi, frame_gray, frame_tresh;
int x_val = 180;
int y_val = 40;
int height = 120;
int width = 240;
int result = x_val;
int car;
int result_193 = 235;
int result_27 = 237;;


int main(int argc, char** argv)
	{
	if (argc != 2) cout << "Add carnumber(27 or 193) as input" << endl;
	car = atof(argv[1]);
	if (car == 27) cout << "car 27 selected" << endl;
	else if (car == 193) cout << "car 193 selected" << endl;
	else cout << "invalid car number" << endl;

		VideoCapture cap(0); // open the default camera
		if (!cap.isOpened()) return -1;
		// cap.set(CV_CAP_PROP_POS_FRAMES, 1); //Set index to first frame
		while (1)
		{
		
		cap.grab();
		cap.retrieve(frame, CV_CAP_OPENNI_BGR_IMAGE);
		result = x_val;

			Rect roi = Rect(x_val, y_val, width, height);
			frame_roi = frame(roi);
			// cv::imshow("roi", frame_roi);
			cvtColor(frame_roi, frame_gray,  COLOR_BGR2GRAY);
			// cv::imshow("gray", frame_gray);
			threshold(frame_gray, frame_tresh, 120, 255, 1);
			// cv::imshow("tresh", frame_tresh);
			int j = frame_tresh.cols / 2;

			for (int i = 0; i < frame_tresh.rows; i++)
			{	
				if (frame_tresh.at <uchar>(i,j) < 230)
				{
					result += i;
					int diff_27 = result - result_27;
					int diff_193 = result - result_193;
					if (car == 27 && abs(diff_27) <= 3)
					{
						cout << "camera position fine (" << result << ")" << endl;
						return 1;
					}
					else if (car == 27 && diff_27 > 3) cout << "lower camera by " << diff_27 << " (" << result << ")" << endl;
					else if (car == 27 && diff_27 < -3) cout << "up camera by " << abs(diff_27) << " (" << result << ")"<< endl;
					else if (car == 193 && abs(diff_193) < 3)
					{
						cout << "camera position fine (" << result << ")" << endl;
						// return 1;
					}
					else if (car == 193 && diff_193 > 3) cout << "lower camera by " << diff_193 << " (" << result << ")"<< endl;
					else if (car == 193 && diff_193 < -3) cout << "up camera by " << abs(diff_193) << " (" << result << ")"<< endl;
					else cout << "error" << " (" << result << ")" << endl;
					break;
				}
			}
		}
		return 0;
	}
