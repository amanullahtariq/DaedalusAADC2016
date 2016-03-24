#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include "DAEDALUS_util.h"
#include "daedalus_action_enums.h"
#include "constant.h"
#include "ParkingType.h"
#include "IPM.h"

using namespace std;
using namespace cv;


float threshValue = 100.0;
float BLOB_BOUND = 5.0;
float BLOB_SUPPORT = 30;
float MIN_HORIZONTAL_LINE_DISTANCE = 100.0;
float MAX_HORIZONTAL_LINE_DISTANCE = 350.0;
float camOffset = 0;

int ParkingType::CrossedParking(Mat inputImage, int cameraOffset)
{
	int result = NOT_FOUND;
	//float threshValue = 100.0;
	Mat gray, binary;
	Mat outputImg;
	
	GenerateIPM(inputImage, outputImg);
		
	cvtColor(outputImg, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(3, 3), 0, 0);
	gray = DAEDALUS::correctGamma(gray, 0.4);

	result = IsCrossedParking(gray);
	camOffset = cameraOffset;
	return result;
}

int ParkingType::CrossedParking(Mat inputImage)
{
	int result = NOT_FOUND;
	//float threshValue = 100.0;
	Mat gray, binary;
	Mat outputImg;

	GenerateIPM(inputImage, outputImg);

	cvtColor(outputImg, gray, CV_BGR2GRAY);
	GaussianBlur(gray, gray, Size(3, 3), 0, 0);
	gray = DAEDALUS::correctGamma(gray, 0.4);

	result = IsCrossedParking(gray);
	return result;
}


void ParkingType::GenerateIPM(Mat inputImg, Mat &outputImg)
{
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(-100, HEIGHT - 100 - camOffset));
	origPoints.push_back(Point2f(WIDTH + 100, HEIGHT - 100 - camOffset));
	origPoints.push_back(Point2f(WIDTH / 2 + 180, 250 - (camOffset * 0.5)));
	origPoints.push_back(Point2f(WIDTH / 2 - 220, 250 - (camOffset * 0.5)));


	 //The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(0, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH, HEIGHT));
	dstPoints.push_back(Point2f(WIDTH , 0));
	dstPoints.push_back(Point2f(0, 0));

	//IPM object
	IPM ipm(Size(WIDTH, HEIGHT), Size(WIDTH , HEIGHT), origPoints, dstPoints);

	Mat inputImgGray;
	// Color Conversion
	if (inputImg.channels() == 3)
		cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);
	else
		inputImg.copyTo(inputImgGray);

	ipm.applyHomography(inputImg, outputImg);
	//ipm.drawPoints(origPoints, inputImg);
	//imshow("outputImg", outputImg);
	//waitKey(0);
}


std::vector<Blob> ParkingType::detectBlobs(Mat &img, float bounds)
{
	std::vector<Blob> _blobs;
	//float bounds = 3.0;

	for (int y = 0; y < img.rows; y++)
	{
		for (int x = 0; x < img.cols; x++)
		{
			if (img.at<uchar>(y, x) >= 200.0)  // is pixel valid?
			{
				bool blobFound = false;
				for (int i = 0; i < _blobs.size(); i++)
				{
					if (abs(x - _blobs.at(i).x) <= bounds &&
						abs(y - _blobs.at(i).y) <= bounds)
					{
						_blobs.at(i).x = ((_blobs.at(i).x * _blobs.at(i).support) + x) / (_blobs.at(i).support + 1);
						_blobs.at(i).y = ((_blobs.at(i).y * _blobs.at(i).support) + y) / (_blobs.at(i).support + 1);
						_blobs.at(i).support++;
						blobFound = true;
						break;
					}
				}
				if (!blobFound)
				{
					Blob b;
					b.x = x;
					b.y = y;
					b.support = 1;
					_blobs.push_back(b);
				}
			}
		}
	}

	return _blobs;
}

std::vector<Blob> ParkingType::detectHorLine(std::vector<Blob> _pts)
{
	std::vector<Blob> _horLines;
	std::vector<Blob> _ret;
	for (int i = 0; i < _pts.size(); i++)
	{
		bool inserted = false;
		for (int j = 0; j < _horLines.size(); j++)
		{
			if (_pts.at(i).y - _horLines.at(j).y < 5)
			{
				_horLines.at(j).x = ((_horLines.at(j).x * _horLines.at(j).support) + _pts.at(i).x) / (_horLines.at(j).support + 1);
				_horLines.at(j).y = ((_horLines.at(j).y * _horLines.at(j).support) + _pts.at(i).y) / (_horLines.at(j).support + 1);
				_horLines.at(j).support++;
				inserted = true;
				break;
			}
		}
		if (!inserted)
		{
			Blob _hLine;
			_hLine.x = _pts.at(i).x;
			_hLine.y = _pts.at(i).y;
			_hLine.support = 1;
			_horLines.push_back(_hLine);
		}
	}

	for (int i = 0; i < _horLines.size(); i++)
	{
		if (_horLines.at(i).support > 15)
			_ret.push_back(_horLines.at(i));
	}

	return _ret;
}

void ParkingType::FindMinMax(std::vector<Blob> horizontalLines, Point2f &minPoint, Point2f &maxPoint)
{
	minPoint.x = 0;
	minPoint.y = 0;
	maxPoint.x = 0;
	maxPoint.y = 0;
	
	for (int i = 0; i < horizontalLines.size(); i++)
	{
		if (horizontalLines.at(i).y > maxPoint.y)
		{
			maxPoint.x = horizontalLines.at(i).x;
			maxPoint.y = horizontalLines.at(i).y;
		}
	}
	if (horizontalLines.size() == 2)
	{
		for (int i = 0; i < horizontalLines.size(); i++)
		{
			if (horizontalLines.at(i).y > minPoint.y)
			{
				minPoint.x = horizontalLines.at(i).x;
				minPoint.y = horizontalLines.at(i).y;
			}
		}
	}
	else
	{
		for (int i = 0; i < horizontalLines.size(); i++)
		{
			if (horizontalLines.at(i).y > minPoint.y && horizontalLines.at(i).y < (maxPoint.y - 50))
			{
				minPoint.x = horizontalLines.at(i).x;
				minPoint.y = horizontalLines.at(i).y;
			}
		}
	}
	
}

int  ParkingType::IsCrossedParking(Mat gray)
{
	Mat binary;
	threshold(gray, binary, threshValue, 255.0, THRESH_BINARY);
	int result = NOT_FOUND;
	Mat roi_bin = binary(Rect(0, 0, WIDTH, binary.rows));
	std::vector<Blob> blobs;
	std::vector<Blob> hLines;
	blobs = detectBlobs(roi_bin, BLOB_BOUND);
	hLines = detectHorLine(blobs);

	Point2f minPoint, maxPoint;

	if (hLines.size() > 1)
	{
		FindMinMax(hLines, minPoint, maxPoint);
	}



	/*
	* Adapting Threshold
	*/
	if (blobs.size() > 250)
		threshValue *= 1.3;
	else if (blobs.size() < 40)
		threshValue *= 0.7;

	threshValue = threshValue > 255 ? 255 : threshValue;
	threshValue = threshValue < 0 ? 0 : threshValue;

	float dist = maxPoint.y - minPoint.y;

	if ((hLines.size() > 1 && dist >= MIN_HORIZONTAL_LINE_DISTANCE && dist <= MAX_HORIZONTAL_LINE_DISTANCE) ||  (hLines.size() == 2 && minPoint.x != maxPoint.x && minPoint.y != maxPoint.y ))
	{
	//	cout << "Cross Parking dist: " << dist;
		result = Driver_cross_parking;
	}
    else if (hLines.size() == 1)
	{
	//	cout << "Parallel Parking dist: " << dist;

         result = Driver_parallel_parking;
    }

	return result;
}

