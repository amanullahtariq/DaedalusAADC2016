#ifndef _POINT_TRANSFORMER_H_
#define _POINT_TRANSFORMER_H_

#include "vector2.h"

class PointTransformer {
 public:
  static Vector2 map_to_aerial_view(Vector2 const & camera_point);
  
 private:
  static const int HALF_HEIGHT_ = 240;
  static const int HALF_WIDTH_ = 320;
};

#endif // _POINT_TRANSFORMER_H_
