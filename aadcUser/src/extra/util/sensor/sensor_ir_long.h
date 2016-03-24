#ifndef __SENSOR_IR_LONG_H__
#define __SENSOR_IR_LONG_H__

#include <stdio.h>
#include "sensor_model.h"

class SensorIrLong : public SensorModel {
  public:
    enum IrLongId {
      FRONT_LEFT,
      FRONT_RIGHT,
      FRONT_CENTER
    };
  
    SensorIrLong(IrLongId id, size_t history_size);
  
  private:
    IrLongId id_;
};

#endif  // __SENSOR_IR_LONG_H__
