#include "sensor_us.h"

// -------------------------------------------------------------------------------------------------
SensorUs::SensorUs(SensorUs::UsId id, size_t history_size) : SensorModel(history_size) {
// -------------------------------------------------------------------------------------------------
  id_ = id;
  
  if (id_ == FRONT_LEFT) {
    origin_x_ = -7;
    origin_y_ = 0;
    angle_ = 105;
  }
  
  else if (id_ == FRONT_RIGHT) {
    origin_x_ = 7;
    origin_y_ = -0;
    angle_ = 75;
  }
  
  else if (id_ == REAR_LEFT) {
    origin_x_ = -7;
    origin_y_ = -58;
    angle_ = 255;
  }

  else if (id_ == REAR_RIGHT) {
    origin_x_ = 7;
    origin_y_ = -58;
    angle_ = 285;
  }
}
