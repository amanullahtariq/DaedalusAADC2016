#ifndef _HISTORY_H_
#define _HISTORY_H_

#include "history_entry.h"
#include <vector>
#include <iostream>
#include <stdio.h>

class History {
 public:
  History(){};
  bool insert(HistoryEntry new_entry);
  int left_get_weighted_x();
  double left_get_weighted_slope();
  double left_get_length();
  int get_left_x();
  int get_left_y();
  bool left_empty();
  int mid_get_weighted_x();
  double mid_get_weighted_slope();
  double mid_get_length();
  int get_mid_x();
  int get_mid_y();
  bool mid_empty();
  int mid_next_get_weighted_x();
  double mid_next_get_weighted_slope();
  bool mid_next_empty();
  int mid_prev_get_weighted_x();
  double mid_prev_get_weighted_slope();
  bool mid_prev_empty();
  int right_get_weighted_x();
  double right_get_weighted_slope();
  double right_get_length();
  int get_right_x();
  int get_right_y();
  bool right_empty();
  int get_size();
  bool jumpSegment();
  
  
  
 
 private:
  std::vector<HistoryEntry> entries_;
  static const int history_size_ = 10;
};

#endif // _HISTORY_H_
