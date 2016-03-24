#include <stdlib.h>   
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include "daedalus_action_enums.h"
#include "constant.h"
#include "ParkingType.h"


float threshValue = 150.0;
float BLOB_BOUND = 5.0;
float BLOB_SUPPORT = 30;
float MIN_HORIZONTAL_LINE_DISTANCE = 100.0;
float MAX_HORIZONTAL_LINE_DISTANCE = 265.0;

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

	if ((hLines.size() > 1 && dist >= MIN_HORIZONTAL_LINE_DISTANCE && dist <= MAX_HORIZONTAL_LINE_DISTANCE) || hLines.size() == 2)
	{
		result = CROSS_PARKING;
	}
  //  else if (hLines.size() == 1 || dist > 0 && dist < 90)
   // {
  //      result = PARALLEL_PARKING;
   // }

	return result;
}

