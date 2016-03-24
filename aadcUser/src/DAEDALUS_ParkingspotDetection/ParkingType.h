#ifndef __PARKING_TYPE_HEADER__
#define __PARKING_TYPE_HEADER__

#include "stdafx.h"


class ParkingType
{
public:	
	void GetParkingInfo(Mat image, bool isParallelParking,bool &result, float &distance,float &horizontalDistance);
	
private:
	void ApplyClahe(Mat inputImage, Mat &outputImage, int limit);
};

#endif /*__DETECT_HORIZONTAL_LINES_H__*/
