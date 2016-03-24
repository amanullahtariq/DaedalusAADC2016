#include "object_map.h"

// -------------------------------------------------------------------------------------------------
ObjectMap::ObjectMap(size_t width, size_t height, size_t cell_size, int cost) 
  : SearchMap(width, height, cell_size, cost) {
// -------------------------------------------------------------------------------------------------
  // Initialize the arrays representing rows in the map
  object_update_array_ = new bool*[height];

  // Fill the row elements with actual row arrays
  for (size_t i = 0; i < height; i++) {
    object_update_array_[i] = new bool[width];
    for (size_t j = 0; j < width; j++) {
      object_update_array_[i][j] = false;
    }
  }
}

// -------------------------------------------------------------------------------------------------
ObjectMap::~ObjectMap() {
// -------------------------------------------------------------------------------------------------
  // Delete the column arrays
  for (size_t i = 0; i < get_map_height(); i++) {
    delete[] object_update_array_[i];
  }

  // Delete the row arrays
  delete[] object_update_array_;
}

// -------------------------------------------------------------------------------------------------
void ObjectMap::update(std::vector<Object> const & objects) {
// -------------------------------------------------------------------------------------------------
  resetArray();
  for (std::vector<Object>::const_iterator obj = objects.begin(); obj != objects.end(); ++obj) {
    size_t row = CAR_ROW - obj->get_absolute_y() / (int)get_cell_size();
    for (int x = 0; x < obj->get_absolute_width(); x += get_cell_size()) {
      size_t col = CAR_COL + (obj->get_absolute_x() + x) / (int)get_cell_size();
      //std::cout << "[Map] (Row, Col): (" << row << ", " << col << ")" << std::endl;
      if (row >= 0 && row < get_map_height() && col >= 0 && col < get_map_width()) {
        object_update_array_[row][col] = true;
        map_array_[row][col]->update(1);
      }
    }    
  }
  for (size_t row = 0; row < CAR_ROW; row++) {
    for (size_t col = 0; col < get_map_width(); col++) if (!object_update_array_[row][col]) map_array_[row][col]->update(0);
  }
}

// -------------------------------------------------------------------------------------------------
void ObjectMap::draw(cv::Mat & dst) const {
// -------------------------------------------------------------------------------------------------
  // Iterate each pixel and add the occupancy value in relation to the scale from 0 to 255
  for (size_t row = 0; row < get_map_height(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      if (!object_update_array_[row][col]) continue; 
      dst.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 0);
      dst.at<cv::Vec3b>(row, col) *= (map_array_[row][col]->occupancy());
    }
  }
}

// -------------------------------------------------------------------------------------------------
void ObjectMap::resetArray() {
// -------------------------------------------------------------------------------------------------
  for (size_t row = 0; row < get_map_height(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      object_update_array_[row][col] = false;
    }
  }
}
