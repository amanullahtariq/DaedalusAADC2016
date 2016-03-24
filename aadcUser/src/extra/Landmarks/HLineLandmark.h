#ifndef _HLINE_LANDMARK_H
#define _HLINE_LANDMARK_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.hpp"
#include "opencv2/core.hpp"
#include "Landmark.h"

using namespace cv;

class HLineLandmark : public Landmark
{
  public:
    HLineLandmark();
    HLineLandmark(Vec3d u, Matx33d Q);
    ~HLineLandmark();
    
    Vec3d		mean;
    Matx33d		covariance;
    int			age;
    double		orientation;
};

#endif // _HLINE_LANDMARK_H
