#ifndef __SENSOR_MODEL_H__
#define __SENSOR_MODEL_H__

#include "../vector2.h"
#include <vector>
#include <stdio.h>

class SensorModel {
  public:
    SensorModel(size_t history_size);
    virtual ~SensorModel();
    void insert(float data_point);
    virtual std::vector<Vector2> getPassed(size_t cell_size) const;
    virtual std::vector<Vector2> getBlocked(size_t cell_size) const;
    
  protected:
    float mean() const;
    int origin_x_;
    int origin_y_;
    int angle_;
    
  private:
    float *history_;
    size_t history_size_, index_;
    bool filled_;
};

#endif  // __SENSOR_MODEL_H__
