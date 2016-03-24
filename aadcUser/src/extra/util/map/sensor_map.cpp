#include "sensor_map.h"

// -------------------------------------------------------------------------------------------------
SensorMap::SensorMap(size_t width, size_t height, size_t cell_size, int cost) 
  : SearchMap(width, height, cell_size, cost) {}
// -------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------
void SensorMap::update(SensorModel & model, float data) {
// -------------------------------------------------------------------------------------------------
  //std::cout << "Update sensor" << std::endl;
  model.insert(data);
  std::vector<Vector2> passed = model.getPassed(get_cell_size());
  std::vector<Vector2> blocked = model.getBlocked(get_cell_size());
  
  for (std::vector<Vector2>::iterator el = passed.begin(); el != passed.end(); ++el) {
    size_t row = get_map_center_row() - el->get_y();
    size_t col = get_map_center_col() + el->get_x();
    if (row >= 0 && row < get_map_height() && col >= 0 && col < get_map_width()) map_array_[row][col]->update(1);
  }

  for (std::vector<Vector2>::iterator el = blocked.begin(); el != blocked.end(); ++el) {
    size_t row = get_map_center_row() - el->get_y();
    size_t col = get_map_center_col() + el->get_x();
    if (row >= 0 && row < get_map_height() && col >= 0 && col < get_map_width()) map_array_[row][col]->update(0);
  }
}

// -------------------------------------------------------------------------------------------------
void SensorMap::draw(cv::Mat & dst) const {
// -------------------------------------------------------------------------------------------------
  // Iterate each pixel and add the occupancy value in relation to the scale from 0 to 255
  for (size_t row = 0; row < get_map_height(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      dst.at<cv::Vec3b>(row, col) = cv::Vec3b(255, 255, 255);
      dst.at<cv::Vec3b>(row, col) *= (map_array_[row][col]->occupancy());
    }
  }
}
