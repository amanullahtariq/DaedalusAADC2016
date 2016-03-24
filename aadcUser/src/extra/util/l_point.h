#ifndef _L_POINT_H_
#define _L_POINT_H_

class LPoint {
 public:
 	LPoint();
 	LPoint(int x1, int x2, int y);
  LPoint(const LPoint& other);
  
  int operator[](unsigned int index) const;
  
  int get_mean_x() const;
 	int get_x1() const;
 	int get_x2() const;
 	int get_y() const;
 
 private:
 	int x1_;
 	int x2_;
 	int y_;
};

#endif // _L_POINT_H_
