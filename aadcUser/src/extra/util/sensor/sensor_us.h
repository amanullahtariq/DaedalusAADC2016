#ifndef __SENSOR_US_H__
#define __SENSOR_US_H__

#include <stdio.h>
#include "sensor_model.h"

class SensorUs : public SensorModel {
  public:
    enum UsId {
      FRONT_LEFT,
      FRONT_RIGHT,
      REAR_LEFT,
      REAR_RIGHT
    };
    
    SensorUs(SensorUs::UsId id, size_t history_size);
  
  private:
    UsId id_;
};

#endif  // __SENSOR_US_H__
