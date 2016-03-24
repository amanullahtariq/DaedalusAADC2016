#include "steering_control.h"
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "../map/map_settings.h"

// -------------------------------------------------------------------------------------------------
float SteeringControl::angle(std::vector<Vector2> const & plan, float discount_factor) {
// -------------------------------------------------------------------------------------------------
  float angle = 0;
  Vector2 origin(Vector2(MAP_WIDTH/2, MAP_HEIGHT/2));
  Vector2 last = origin.computeVector(plan[plan.size() - 1]);

  float discount[] = {.7, .2, .06, .04};
  
  /*for (int i = plan.size() - 1; i >= 1; i--) {
    std::cout << "(" << plan[i].get_x() << ", " << plan[i].get_y() << ") - ";
  }
  std::cout << "(" << plan[0].get_x() << ", " << plan[0].get_y() << ")" << std::endl;*/
  
  for (int i = plan.size() - 1; i >= 1; i--) {
    Vector2 current_vector = plan[i].computeVector(plan[i-1]);
    float current_angle = last.computeAngle(current_vector);
    
    if (abs(current_angle) > 90) {
      if (current_angle < 0) current_angle += 180;
      else current_angle -= 180;
    }
    
    if (plan[i-1].get_x() < plan[i].get_x()) current_angle *= -1;
    
    //if (last.isLeftTurn(current_vector)) current_angle *= -1;
    last = current_vector;
    
    /*std::cout << "i=(" << plan[i].get_x() << ", " << plan[i].get_y() << ")" << std::endl;
    std::cout << "i-1=(" << plan[i-1].get_x() << ", " << plan[i-1].get_y() << ")" << std::endl;
    std::cout << "Current Angle : " << current_angle << std::endl;
    std::cout << "---" << std::endl;*/
    angle += current_angle * discount[plan.size()-1-i];
  }
  return 30 * angle/90.0;
}
