#ifndef __SEARCH_MAP_H__
#define __SEARCH_MAP_H__

#include "../vector2.h"
#include "../map/map.h"
#include <stdlib.h>
#include <vector>
#include <list>

class SearchMap : public Map {
  public:
    SearchMap(size_t map_width, size_t map_height, size_t cell_size, int cost);

    int get_cost_value() const;
    double get_cost(Vector2 position) const;
    double get_cost(Vector2 position, int width, int height) const;
    
  private:
    int cost_;
};

#endif  // __SEARCH_MAP_H__
