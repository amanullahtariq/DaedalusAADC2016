#ifndef __OBJECT_MAP_H_
#define __OBJECT_MAP_H_

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <string>

#include "../vector2.h"
#include "../object.h"
#include "../gridSearch/SearchMap.h"
#include "cell.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ObjectMap : public SearchMap {
  public:
    ObjectMap(size_t width, size_t height, size_t cell_size, int cost);
    ~ObjectMap();

    void update(std::vector<Object> const & objects);
    void draw(cv::Mat & dst) const;
    
  private:
    void resetArray();
    
    bool** object_update_array_;
};

#endif  // __OBJECT_MAP_H_
