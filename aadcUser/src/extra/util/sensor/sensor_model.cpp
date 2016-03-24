#include "sensor_model.h"
#include "sensor_beam.h"
#include <iostream>

// -------------------------------------------------------------------------------------------------
SensorModel::SensorModel(size_t history_size) {
// -------------------------------------------------------------------------------------------------
  history_size_ = history_size;
  history_ = new float[history_size_];
  index_ = 0;
  filled_ = false;
}

// -------------------------------------------------------------------------------------------------
SensorModel::~SensorModel() {
// -------------------------------------------------------------------------------------------------
  delete[] history_;
}

// -------------------------------------------------------------------------------------------------
void SensorModel::insert(float data_point) {
// -------------------------------------------------------------------------------------------------
  history_[index_++] = data_point;
  if (index_ >= history_size_) 
  {
    filled_ = true;
    index_ = 0;
  }
}

// -------------------------------------------------------------------------------------------------
float SensorModel::mean() const {
// -------------------------------------------------------------------------------------------------
  float sum = 0;
  for (size_t i = 0; i < (filled_ ? history_size_ : index_); i++) sum += history_[i];
  return sum/history_size_;
}

// -------------------------------------------------------------------------------------------------
std::vector<Vector2> SensorModel::getPassed(size_t cell_size) const {
// -------------------------------------------------------------------------------------------------
  SensorBeam b(Vector2(origin_x_, origin_y_), angle_, mean());
  std::vector<Vector2> line = b.getLinePoints();
  std::vector<Vector2> result;
  for (size_t i = 0; i < line.size(); i++) result.push_back(Vector2(line.at(i).get_x() / (int)cell_size, line.at(i).get_y() / (int)cell_size));
  return result;
}

// -------------------------------------------------------------------------------------------------
std::vector<Vector2> SensorModel::getBlocked(size_t cell_size) const {
// -------------------------------------------------------------------------------------------------
  SensorBeam b(Vector2(origin_x_, origin_y_), angle_, mean());
  std::vector<Vector2> result;
  result.push_back(Vector2(b.getEnd().get_x() / (int)cell_size, b.getEnd().get_y() / (int)cell_size));
  return result;
}
