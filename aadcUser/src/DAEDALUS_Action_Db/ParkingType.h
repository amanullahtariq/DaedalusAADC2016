#ifndef __PARKING_TYPE_HEADER__
#define __PARKING_TYPE_HEADER__

#include <opencv/cv.h>
using namespace cv;

struct Blob
{
	int support;
	float x;
	float y;
};

class ParkingType
{
public:	
	int IsCrossedParking(Mat image);
private:
	std::vector<Blob> detectBlobs(Mat &img, float bounds);
	std::vector<Blob> detectHorLine(std::vector<Blob> _pts);
	void FindMinMax(std::vector<Blob> horizontalLines, Point2f &minPoint, Point2f &maxPoint);
	
};

#endif /*__DETECT_HORIZONTAL_LINES_H__*/