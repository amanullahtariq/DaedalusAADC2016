#include "stdafx.h"
#include "ParkingType.h"


float thresholdValue = 50.0;
float A_BLOB_BOUND = 5.0;
float A_BLOB_SUPPORT = 30;
float MIN_HORIZONTAL_LINE_DISTANCE = 100.0;
float MAX_HORIZONTAL_LINE_DISTANCE = 265.0;

float threshValue = 230;
float BLOB_BOUND = 4;
float BLOB_SUPPORT = 3;
float BLOB_BOUND_P = 4;
float BLOB_SUPPORT_P = 3;
float MIN_DISTANCE_CROSS = 75.0;
float MAX_DISTANCE_CROSS = 95.0;
float MIN_DISTANCE_PARALLEL = 75.0;
float MAX_DISTANCE_PARALLEL = 95.0;



void ParkingType:: ApplyClahe(Mat inputImage, Mat &outputImage, int limit)
{
	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(limit);
	clahe->apply(inputImage, outputImage);
}


void ParkingType::GetParkingInfo(Mat image, bool isParallelParking,bool &result, float &line_distance, float &distance)
{
	Mat gray, binary, blob,roi;
    if (isParallelParking)
        {
            roi = image(Rect(375, 100, 55, 480 - 100));
            BLOB_BOUND = BLOB_BOUND_P;
            BLOB_SUPPORT = BLOB_SUPPORT_P;
        }
	else roi = image(Rect(375, 100, 80, 480 - 100));

	cvtColor(roi, gray, CV_BGR2GRAY);
	ApplyClahe(gray, gray, 4);
	threshold(gray, binary, threshValue, 255.0, THRESH_BINARY);
	cvtColor(binary, blob, CV_GRAY2BGR);
	std::vector<DAEDALUS::Blob> blobs;
	std::vector<DAEDALUS::Blob> hLines;

	blobs = DAEDALUS::detectBlobs(binary, BLOB_BOUND);
	hLines = DAEDALUS::detectHorLine(blobs, BLOB_SUPPORT);

    // draw blobs into image
	/*for (int j = 0; j < blobs.size(); j++)
	{
			circle(blob, Point2d(blobs.at(j).x, blobs.at(j).y), 5, Scalar(255, 0, 0));
	}
    // draw hlines into image
	for (int j = 0; j < hLines.size(); j++)
	{
		for (int k = 0; k < 64; k++)
		{
			circle(blob, Point2d(k * 10, hLines.at(j).y), 5, Scalar(0, 255, 0));
		}
	}*/
	//imshow("blobs", blob);

	if (hLines.size() > 1)
	{
		for (int h = hLines.size() - 1; h > 0; h--)
		{
			line_distance = abs(hLines.at(h).y - hLines.at(h - 1).y);
			if (isParallelParking  && line_distance >= MIN_DISTANCE_PARALLEL && line_distance <= MAX_DISTANCE_PARALLEL)
			{
				result = true;
				distance = 480 - hLines.at(h).y;
				//printf("result = %d hLines.size = %d, dist = %f, line_distance = %f\n", result, hLines.size(), distance, line_distance);
			}
			else if (!isParallelParking  && line_distance >= MIN_DISTANCE_CROSS && line_distance <= MAX_DISTANCE_CROSS)
			{
				result = true;
				distance = 480 - hLines.at(h).y;
				//printf("result = %d hLines.size = %d, dist = %f, line_distance = %f\n", result, hLines.size(), distance, line_distance);
			}
			else
			{
				result = false;
				distance = 0;
				//printf("result = %d hLines.size = %d, dist = %f, line_distance = %f\n", result, hLines.size(), distance, line_distance);
			}

		}
	}

	//printf("result = %d hLines.size = %d, dist = %f, blobs = %d, tresh = %f \n", result, hLines.size(), distance, blobs.size(), threshValue);
}
