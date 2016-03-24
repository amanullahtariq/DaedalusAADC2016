#ifndef __LANE_MAP_H_
#define __LANE_MAP_H_

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <string>

#include "../point_transformer.h"
#include "../vector2.h"
#include "../gridSearch/SearchMap.h"
#include "cell.h"


#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class LaneMap : public SearchMap {
  public:
    LaneMap(size_t width, size_t height, size_t cell_size, int cost);
    ~LaneMap();

    void update(std::vector<Vector2> const & lane_points);
    void setOccupancy(Vector2 location, double value);
    void draw(cv::Mat & dst) const;

  private:
    void resetArray();
    double cone(int x);
    void initializeCone();
    //std::vector<Vector2> cone_points_;
    int cone_points_[MAP_HEIGHT][2];
    bool check_cone_boundaries(int col, int row);
    const static double cone_offset_x_ = 6;
    const static double cone_offset_y_ = -20.0;
    const static double left_cone_slope_ = 1.75; // 1.65 // 1.67
    const static double right_cone_slope_ = 1.9;
    
    bool** lane_update_array_;
};

#endif  // __LANE_MAP_H_
