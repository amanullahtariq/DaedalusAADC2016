#ifndef __MAP_H__
#define __MAP_H__

#include "../vector2.h"
#include "cell.h"
#include <stdio.h>

class Map {
 public:
  Map(size_t width, size_t height, size_t cell_size);
  ~Map();
  
  size_t get_map_width() const;
  size_t get_map_height() const;
  size_t get_cell_size() const;
  size_t get_map_center_row() const;
  size_t get_map_center_col() const;
  
  float occupancy(Vector2 const & position) const;
  void transform(Vector2** transformation_matrix);
 
 protected:
  Cell*** map_array_;
  
 private:
  size_t width_;
  size_t height_;
  size_t cell_size_;
};

#endif  // __MAP_H__
