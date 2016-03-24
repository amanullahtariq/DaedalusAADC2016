#include "point_transformer.h"
#include <stdlib.h>
#include <math.h>

// -------------------------------------------------------------------------------------------------
Vector2 PointTransformer::map_to_aerial_view(Vector2 const & camera_point) {
// -------------------------------------------------------------------------------------------------
  // Step1:
  // red triangle
  double l_r = sqrt(pow((HALF_WIDTH_ / tan(29 / 180.0 * M_PI)), 2) + pow((camera_point.get_y() - HALF_HEIGHT_), 2) + pow((camera_point.get_x() - HALF_WIDTH_), 2));

  // green triangle
  double l_g = sqrt(pow((HALF_WIDTH_ / tan(29 / 180.0 * M_PI)), 2) + pow((HALF_HEIGHT_), 2) + pow((camera_point.get_x() - HALF_WIDTH_), 2));

  // l_uc
  double l_uc = sqrt(pow((HALF_WIDTH_ / tan(29 / 180.0 * M_PI)), 2) + pow((camera_point.get_x() - HALF_WIDTH_), 2));
  
  // alpha
  double alpha;
  if (camera_point.get_y() >= HALF_HEIGHT_) {
    alpha = acos((pow(l_r, 2) + pow(l_g, 2) - pow((480 - camera_point.get_y()), 2)) / (2 * l_r * l_g));
  } else {
    alpha = acos((pow(l_r, 2) + pow(l_g, 2) - pow(camera_point.get_y(), 2)) / (2 * l_r * l_g));
  }
 
  // Lenth_Bottom
  double l_b = sqrt(pow((HALF_WIDTH_ / tan(29 / 180.0 * M_PI)), 2) + pow(HALF_HEIGHT_, 2));

  // phi
  double phi = atan(abs(camera_point.get_x() - HALF_WIDTH_) / l_b);

  // delta
  double delta = atan(HALF_WIDTH_ / l_b);

  // Rho
  double rho = atan(HALF_HEIGHT_ / l_uc);

  // Step2:

  // real_distance 43           50
  int p = 45;
  // half of real distance
  double q = p / 2.0;

  // l_o
  double l_o = q / tan(delta);

  // l_l
  double l_l = tan(phi) * l_o;

  // l_rbo   original delta is 30 (30.55), modify to change camera angle  29.1
  double l_rbo = q / tan(30.55 / 180.0 * M_PI);

  // l_rbg
  double l_rbg = sqrt(pow(l_rbo, 2) + pow(l_l, 2));

  // l_ig
  double l_ig = l_o / cos(phi);

  // l_vb
  double l_vb = sqrt(pow(l_o, 2) - pow(l_rbo, 2));

  // theta
  double theta = acos(l_vb / l_ig);

  // Step3:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // ksi
  double ksi;
  if (camera_point.get_y() >= HALF_HEIGHT_) {
    ksi = alpha + theta;
  } else {
    ksi = (2 * rho - alpha) + theta;
   }

  // l_rd
  double l_rd = l_vb * tan(ksi);

  // Step4:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  double l_r2 = l_l * l_rd / l_rbg;
  double real_x;
  if((atan((camera_point.get_x() - HALF_WIDTH_) / l_b)) > 0) {
    real_x = l_r2;
  } else {
    real_x = -l_r2;
  }

  double real_y = ((l_rd * l_rbo) / l_rbg - l_rbo);

  return Vector2(real_x - 6, real_y + 20);  
}

