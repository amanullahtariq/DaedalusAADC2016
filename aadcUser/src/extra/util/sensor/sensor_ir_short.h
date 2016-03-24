#ifndef __SENSOR_IR_SHORT_H__
#define __SENSOR_IR_SHORT_H__

#include <stdio.h>
#include "sensor_model.h"

class SensorIrShort : public SensorModel {
  public:
    enum IrShortId {
      FRONT_CENTER,
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_CENTER,
      LEFT,
      RIGHT
    };
  
    SensorIrShort(SensorIrShort::IrShortId id, size_t history_size);
  
  private:
    SensorIrShort::IrShortId id_;
};

#endif  // __SENSOR_IR_SHORT_H__
