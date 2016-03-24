#ifndef _OBJECT_H_
#define _OBJECT_H_

class Object {
 public:
  Object();
  Object(int x, int y, int width, int height, int source_height, int source_width);
  
  int get_absolute_x() const;
  int get_absolute_y() const;
  int get_absolute_width() const;
  int get_absolute_height() const;
  
  double get_relative_x() const;
  int get_relative_x(int actual_width) const;
  double get_relative_y() const;
  int get_relative_y(int actual_height) const;
  double get_relative_width() const;
  int get_relative_width(int actual_width) const;
  double get_relative_height() const;
  int get_relative_height(int actual_height) const;
  
  static const int ERROR_VALUE = -999;

 private:
  int x_;
  int y_;
  int width_;
  int height_;
  int source_width_;
  int source_height_;
};

#endif // _OBJECT_H_
