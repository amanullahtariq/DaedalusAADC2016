#include "map.h"
#include "map_settings.h"
#include <iostream>

// -------------------------------------------------------------------------------------------------
Map::Map(size_t width, size_t height, size_t cell_size) {
// -------------------------------------------------------------------------------------------------
  // Set the member variables
  width_ = width;
  height_ = height;
  cell_size_ = cell_size;
  
  // Initialize the cell array
  map_array_ = new Cell**[height];
  for (size_t row = 0; row < height; row++) {
    map_array_[row] = new Cell*[width];
    for (size_t col = 0; col < width; col++) {
      map_array_[row][col] = new Cell(HISTORY_SIZE, DISCOUNT_FACTOR);
    }
  }
}

// -------------------------------------------------------------------------------------------------
Map::~Map() {
// -------------------------------------------------------------------------------------------------
  for (size_t row = 0; row < height_; row++) {
    for (size_t col = 0; col < width_; col++) {
      delete map_array_[row][col];
    }
    delete[] map_array_[row];
  }
  delete[] map_array_;
}

// -------------------------------------------------------------------------------------------------
size_t Map::get_map_width() const {
// -------------------------------------------------------------------------------------------------
  return width_;
}

// -------------------------------------------------------------------------------------------------
size_t Map::get_map_height() const {
// -------------------------------------------------------------------------------------------------
  return height_;
}

// -------------------------------------------------------------------------------------------------
size_t Map::get_cell_size() const {
// -------------------------------------------------------------------------------------------------
  return cell_size_;
}

// -------------------------------------------------------------------------------------------------
size_t Map::get_map_center_row() const {
// -------------------------------------------------------------------------------------------------
  return height_ / 2;
}

// -------------------------------------------------------------------------------------------------
size_t Map::get_map_center_col() const {
// -------------------------------------------------------------------------------------------------
  return width_ / 2;
}
 
// -------------------------------------------------------------------------------------------------
float Map::occupancy(Vector2 const & position) const {
// -------------------------------------------------------------------------------------------------
  return map_array_[position.get_y()][position.get_x()]->occupancy();
}

// -------------------------------------------------------------------------------------------------
void Map::transform(Vector2** transformation_matrix) {
// -------------------------------------------------------------------------------------------------
  Cell*** new_map_array = new Cell**[height_];
  for (size_t row = 0; row < height_; row++) {
    new_map_array[row] = new Cell*[width_];
    for (size_t col = 0; col < width_; col++) {
      new_map_array[row][col] = new Cell(HISTORY_SIZE, DISCOUNT_FACTOR);
    }
  }

  for (size_t row = 0; row < height_; row++) {
    for (size_t col = 0; col < width_; col++) {
      int transformed_x = transformation_matrix[row][col].get_x();
      int transformed_y = transformation_matrix[row][col].get_y();
      
      if (transformed_x != Vector2::ERROR_VALUE && transformed_y != Vector2::ERROR_VALUE) {
        Cell* tmp = new_map_array[transformed_y][transformed_x];
        new_map_array[transformed_y][transformed_x] = map_array_[row][col];
        map_array_[row][col] = tmp;
      }
    }
  }
  
  
  float updated_occupancies[height_][width_];
  for (size_t row = 0; row < height_; row++) {
    for (size_t col = 0; col < width_; col++) {
      if (new_map_array[row][col]->occupancy() != INITIAL_OCCUPANCY) continue;
      double updated_occupancy = 0.0;
      int blur_counter = 0;
      for (int x_offset = -1; x_offset <= 1; x_offset++) {
        for (int y_offset = -1; y_offset <= 1; y_offset++) {
          if (x_offset == 0 && y_offset == 0) continue;
          int new_row = row + y_offset;
          int new_col = col + x_offset;
          if (new_row < 0) new_row = -new_row;
          if (new_col < 0) new_col = -new_col;
          if (new_row >= (int)get_map_height()) new_row = new_row - (int)get_map_height();
          if (new_col >= (int)get_map_width()) new_col = new_col - (int)get_map_width();
          updated_occupancy += new_map_array[new_row][new_col]->occupancy();
          blur_counter++;
        }
      }
      updated_occupancy /= blur_counter;
      updated_occupancies[row][col] = updated_occupancy;
    }
  }
  
  for (size_t row = 0; row < height_; row++) {
    for (size_t col = 0; col < width_; col++) {
      if (new_map_array[row][col]->occupancy() != 0.5) continue;
      for (size_t i = 0; i < new_map_array[row][col]->history_size(); i++) new_map_array[row][col]->update(updated_occupancies[row][col]);
    }
  }
  
  Cell*** tmp = map_array_;
  map_array_ = new_map_array;
  
  for (size_t row = 0; row < height_; row++) {
    for (size_t col = 0; col < width_; col++) {
      delete tmp[row][col];
    }
    delete[] tmp[row];
  }
  delete[] tmp;
}
