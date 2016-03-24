#ifndef _LANDMARK_H
#define _LANDMARK_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.hpp"
#include "opencv2/core.hpp"

using namespace cv;

class Landmark
{
  public:
    Landmark();
    Landmark(Vec3d u, Matx33d Q);
    ~Landmark();
    
    Vec3d		mean;
    Matx33d		covariance;
    int			age;
    double		fitness;
};

#endif // _LANDMARK_H
