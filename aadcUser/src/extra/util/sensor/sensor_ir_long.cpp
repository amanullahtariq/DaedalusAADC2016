#include "sensor_ir_long.h"

#include <iostream>

// -------------------------------------------------------------------------------------------------
SensorIrLong::SensorIrLong(IrLongId id, size_t history_size) : SensorModel(history_size) {
// -------------------------------------------------------------------------------------------------
  id_ = id;
  
  if (id_ == FRONT_CENTER) {
    origin_x_ = 0;
    origin_y_ = 0;
    angle_ = 90;
  }
  
  else if (id_ == FRONT_LEFT) {
    origin_x_ = -13;
    origin_y_ = -1;
    angle_ = 180;
  }
  
  else if (id_ == FRONT_RIGHT) {
    origin_x_ = 13;
    origin_y_ = -1;
    angle_ = 0;
  }
}
