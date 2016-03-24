#ifndef __SENSOR_MAP_H_
#define __SENSOR_MAP_H_

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <string>

#include "../vector2.h"
#include "../sensor/sensor_model.h"
#include "../gridSearch/SearchMap.h"
#include "cell.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class SensorMap : public SearchMap {
  public:
    SensorMap(size_t width, size_t height, size_t cell_size, int cost);

    void update(SensorModel & model, float data);
    void draw(cv::Mat & dst) const;
};

#endif  // __SENSOR_MAP_H_
