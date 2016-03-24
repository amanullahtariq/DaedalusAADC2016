#ifndef _HISTORY_ENTRY_H_
#define _HISTORY_ENTRY_H_

class HistoryEntry {
 public:
  HistoryEntry(int left_x, double left_slope, bool left_available_, int left_avg_x, int left_avg_y, double left_length,
               int mid_x, double mid_slope, bool mid_available, int mid_avg_x, int mid_avg_y, double mid_length,
               int mid_next_x, double mid_next_slope, bool mid_next_available,
               int mid_prev_x, double mid_prev_slope, bool mid_prev_available,
               int right_x, double right_slope, bool right_available, int right_avg_x, int right_avg_y, double right_length);
  void passDataOn();
  
  int left_x_;
  double left_slope_;
  int left_avg_x_;
  int left_avg_y_;
  double left_length_;
  bool left_available_;
  int mid_x_;
  double mid_slope_;
  int mid_avg_x_;
  int mid_avg_y_;
  double mid_length_;
  bool mid_available_;
  int mid_next_x_;
  double mid_next_slope_;
  bool mid_next_available_;
  int mid_prev_x_;
  double mid_prev_slope_;
  bool mid_prev_available_;
  int right_x_;
  double right_slope_;
  int right_avg_x_;
  int right_avg_y_;
  double right_length_;
  bool right_available_;
 
 private:

};

#endif // _HISTORY_ENTRY_H_
