#ifndef __TEST_MAP_H__
#define __TEST_MAP_H__

#include "SearchMap.h"
#include <utility>
#include <stdio.h>

class TestMap : public SearchMap {
  public:
    TestMap(float** map_array, int cost, size_t cell_size, int map_width, int map_height);

    float isOccupied(std::pair<int, int> position) const;

  private:
    float** map_array_;
};

#endif