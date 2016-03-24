#ifndef __SENSOR_BEAM_H__
#define __SENSOR_BEAM_H__

#include <vector>
#include "../vector2.h"

class SensorBeam {
  public:
    SensorBeam(Vector2 const & origin, int angle, double length);

    /* 
     * Uses Bresenham's algorithm to determine the points along the sensor beam
     * @return Points of the line beam (without the end point).
     */
    std::vector<Vector2> getLinePoints() const;
    Vector2 getEnd() const;

  private:
    Vector2 origin_;
    int angle_;
    double length_;
};

#endif  // __SENSOR_BEAM_H__