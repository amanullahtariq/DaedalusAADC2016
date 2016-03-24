#ifndef _LINE_TRIPLE_H_
#define _LINE_TRIPLE_H_

#include "l_point.h"
#include <vector>

class LineTriple {
 public:
  LineTriple() {};
 	LineTriple(std::vector<LPoint> leftLine, std::vector<LPoint> middleLine, std::vector<LPoint> rightLine);

 	std::vector<LPoint> get_left() const;
 	std::vector<LPoint> get_mid() const;
 	std::vector<LPoint> get_right() const;
 	void set_left(std::vector<LPoint> leftLine);
 	void set_mid(std::vector<LPoint> middleLine);
 	void set_right(std::vector<LPoint> rightLine);
 
 private:
 	std::vector<LPoint> left_line_;
 	std::vector<LPoint> middle_line_;
 	std::vector<LPoint> right_line_;
};

#endif // _LINE_TRIPLE_H_
