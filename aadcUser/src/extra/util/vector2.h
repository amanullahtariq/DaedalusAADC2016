#ifndef _VECTOR_2_H_
#define _VECTOR_2_H_

#include <string>

class Vector2 {
 public:
  Vector2(int x, int y);
  Vector2(Vector2 const & other);
  Vector2();
  
  bool operator==(Vector2 const & other) const;
  int operator[](unsigned int index) const;
  Vector2 operator+(Vector2 const &other) const;
  Vector2 operator*(double scalar) const;
  int get_x() const;
  int get_y() const;
  Vector2 computeVector(Vector2 const &other) const;
  double computeAngle(Vector2 const &other) const;
  int computeScalar(Vector2 const &other) const;
  double absoluteValue() const;
  bool isLeftTurn(Vector2 const &other) const;
  std::string toString() const;
  
  static const int ERROR_VALUE = -999;
  
 private:
  int x_;
  int y_;
};

#endif  // _VECTOR_2_H_
