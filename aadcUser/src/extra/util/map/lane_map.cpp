#include "lane_map.h"

// -------------------------------------------------------------------------------------------------
LaneMap::LaneMap(size_t width, size_t height, size_t cell_size, int cost) 
  : SearchMap(width, height, cell_size, cost) {
// -------------------------------------------------------------------------------------------------
  // Initialize the arrays representing rows in the map
  lane_update_array_ = new bool*[height];
  initializeCone();
  // Fill the row elements with actual row arrays
  for (size_t i = 0; i < height; i++) {
    lane_update_array_[i] = new bool[width];
    for (size_t j = 0; j < width; j++) {
      lane_update_array_[i][j] = false;
    }
  }
}

// -------------------------------------------------------------------------------------------------
LaneMap::~LaneMap() {
// -------------------------------------------------------------------------------------------------
  // Delete the column arrays
  for (size_t i = 0; i < get_map_height(); i++) {
    delete[] lane_update_array_[i];
  }

  // Delete the row arrays
  delete[] lane_update_array_;
}

// -------------------------------------------------------------------------------------------------
void LaneMap::update(std::vector<Vector2> const & lane_points) {
// -------------------------------------------------------------------------------------------------
  resetArray();
  
  for (std::vector<Vector2>::const_iterator el = lane_points.begin(); el != lane_points.end(); ++el) {
    size_t row = CAR_ROW - el->get_y() / (int)get_cell_size();
    size_t col = CAR_COL + el->get_x() / (int)get_cell_size();
    if (row >= 0 && row < get_map_height() && col >= 0 && col < get_map_width()) {
      map_array_[row][col]->update(1);
      lane_update_array_[row][col] = true;
    }
  }
  
  for (size_t row = 0; row <= CAR_ROW - 30 / (int)get_cell_size(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      if(check_cone_boundaries(col, row)) if (!lane_update_array_[row][col]) map_array_[row][col]->update(0);
    }
  }
  /*
  for (size_t row = 0; row < CAR_ROW - 34 / (int)get_cell_size(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      if((int)row < (int)cone(col) && col < get_map_width() / 2) if (!lane_update_array_[row][col]) map_array_[row][col]->update(0.3);
    }
  }*/
}

// -------------------------------------------------------------------------------------------------
void LaneMap::setOccupancy(Vector2 location, double value) {
// -------------------------------------------------------------------------------------------------
  int row = location.get_y();
  int col = location.get_x();
  if (row >= 0 && row < get_map_height() && col >= 0 && col < get_map_width()) map_array_[row][col]->update(value);
}

// -------------------------------------------------------------------------------------------------
void LaneMap::draw(cv::Mat & dst) const {
// -------------------------------------------------------------------------------------------------
  // Iterate each pixel and add the occupancy value in relation to the scale from 0 to 255
  for (size_t row = 0; row < get_map_height(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      dst.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
      dst.at<cv::Vec3b>(row, col) *= map_array_[row][col]->occupancy();
    }
  }
}

// -------------------------------------------------------------------------------------------------
void LaneMap::resetArray() {
// -------------------------------------------------------------------------------------------------
  for (size_t row = 0; row < get_map_height(); row++) {
    for (size_t col = 0; col < get_map_width(); col++) {
      lane_update_array_[row][col] = false;
    }
  }
}

// -------------------------------------------------------------------------------------------------
double LaneMap::cone(int x) {
// -------------------------------------------------------------------------------------------------
  double cone_slope_ = (x + (cone_offset_x_ / (int)get_cell_size())) < CAR_COL ? left_cone_slope_ : right_cone_slope_;
  return -fabs((x + (cone_offset_x_ / (int)get_cell_size()) - CAR_COL) * cone_slope_) - (cone_offset_y_ / (int)get_cell_size()) + CAR_ROW;
}

// -------------------------------------------------------------------------------------------------
void LaneMap::initializeCone() {
// -------------------------------------------------------------------------------------------------
  for (int i = 0; i < MAP_HEIGHT; i++) {
    cone_points_[i][0] = 999999;
    cone_points_[i][1] = -999999;
  }
  for(size_t row = 192; row < 192 + 225; row++) {
    Vector2 left = Vector2(0, row);
    Vector2 right = Vector2(639, row);
    Vector2 mapped_left = PointTransformer::map_to_aerial_view(left);
    Vector2 mapped_right = PointTransformer::map_to_aerial_view(right);
    mapped_left = Vector2(mapped_left.get_x() / CELL_SIZE + CAR_COL, CAR_ROW - mapped_left.get_y() / CELL_SIZE);
    mapped_right = Vector2(mapped_right.get_x() / CELL_SIZE + CAR_COL, CAR_ROW - mapped_right.get_y() / CELL_SIZE);
    if(mapped_left.get_y() > 0 && mapped_left.get_y() < MAP_HEIGHT)
      if(cone_points_[mapped_left.get_y()][0] > mapped_left.get_x()) cone_points_[mapped_left.get_y()][0] = mapped_left.get_x();
    if(mapped_right.get_y() > 0 && mapped_right.get_y() < MAP_HEIGHT)
      if(cone_points_[mapped_right.get_y()][1] < mapped_right.get_x()) cone_points_[mapped_right.get_y()][1] = mapped_right.get_x();
  }
  for(size_t row = 1; row < MAP_HEIGHT / 2; row++) {
    if (cone_points_[row][0] == 999999) cone_points_[row][0] = (cone_points_[row - 1][0] + cone_points_[row + 1][0]) / 2;
    if (cone_points_[row][1] == -999999) cone_points_[row][1] = (cone_points_[row - 1][1] + cone_points_[row + 1][1]) / 2;
  } 
  for(size_t row = 0; row < MAP_HEIGHT; row++) {
    //std::cout << "Row: " << row << " min: " << cone_points_[row][0] << ", max: " << cone_points_[row][1] << std::endl;
  }
}

// -------------------------------------------------------------------------------------------------
bool LaneMap::check_cone_boundaries(int col, int row) {
// -------------------------------------------------------------------------------------------------
  if(col >= cone_points_[row][0] && col <= cone_points_[row][1] && col > 0 && col < get_map_width()) return true;
  else return false;
}
