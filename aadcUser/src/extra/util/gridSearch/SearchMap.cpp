#include "SearchMap.h"
#include "../map/map_settings.h"
#include <iostream>

// -------------------------------------------------------------------------------------------------
SearchMap::SearchMap(size_t map_width, size_t map_height, size_t cell_size, int cost) 
  : Map(map_width, map_height, cell_size) {
// -------------------------------------------------------------------------------------------------
  cost_ = cost;
}

// -------------------------------------------------------------------------------------------------
int SearchMap::get_cost_value() const {
// -------------------------------------------------------------------------------------------------
  return cost_;
}

// -------------------------------------------------------------------------------------------------
double SearchMap::get_cost(Vector2 position) const {
// -------------------------------------------------------------------------------------------------
  return get_cost(position, CAR_WIDTH, CAR_LENGTH);
}

// -------------------------------------------------------------------------------------------------
double SearchMap::get_cost(Vector2 position, int width, int length) const {
// -------------------------------------------------------------------------------------------------
    double costs = 0;
  int cell_size = (int)get_cell_size();
  // Iterate through every cell of the car
  for (int i = 0; i < (width / cell_size); i++) {
    for (int j = 0; j < (length / cell_size); j++) {
      int cur_x = position.get_x() - ((width / get_cell_size()) / 2) + i;
      int cur_y = position.get_y() + j;
      Vector2 cur(cur_x, cur_y);

      // Retrieve the occupancy probability value and multiply it with the map cost
      if (cur_x < 0 || cur_y < 0 || cur_x >= (int)get_map_width() || cur_y >= (int)get_map_height()) {
        continue;
      }

      costs += (occupancy(cur) * get_cost_value());
    }
  }
  return costs;
}
