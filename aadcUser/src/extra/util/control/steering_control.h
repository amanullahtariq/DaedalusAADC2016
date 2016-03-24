#ifndef __STEERING_CONTROL_H_
#define __STEERING_CONTROL_H_

#include "../vector2.h"
#include <vector>

class SteeringControl {
  public:
    float angle(std::vector<Vector2> const & plan, float discount_factor);
};

#endif // __STEERING_CONTROL_H_
