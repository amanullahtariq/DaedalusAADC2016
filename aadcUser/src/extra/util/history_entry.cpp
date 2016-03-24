#include "history_entry.h"
#include <assert.h>

// -------------------------------------------------------------------------------------------------
HistoryEntry::HistoryEntry(int left_x, double left_slope, bool left_available, int left_avg_x, int left_avg_y, double left_length,
                           int mid_x, double mid_slope, bool mid_available, int mid_avg_x, int mid_avg_y, double mid_length,
                           int mid_next_x, double mid_next_slope, bool mid_next_available,
                           int mid_prev_x, double mid_prev_slope, bool mid_prev_available,
                           int right_x, double right_slope, bool right_available, int right_avg_x, int right_avg_y, double right_length) {
// -------------------------------------------------------------------------------------------------
  left_x_ = left_x;
  left_slope_ = left_slope;
  left_avg_x_ = left_avg_x;
  left_avg_y_ = left_avg_y;
  left_length_ = left_length;
  left_available_ = left_available;
  mid_x_ = mid_x;
  mid_slope_ = mid_slope;
  mid_avg_x_ = mid_avg_x;
  mid_avg_y_ = mid_avg_y;
  mid_length_ = mid_length;
  mid_available_ = mid_available;
  mid_next_x_ = mid_next_x;
  mid_next_slope_ = mid_next_slope;
  mid_next_available_ = mid_next_available;
  mid_prev_x_ = mid_prev_x;
  mid_prev_slope_ = mid_prev_slope;
  mid_prev_available_ = mid_prev_available;
  right_x_ = right_x;
  right_slope_ = right_slope;
  right_avg_x_ = right_avg_x;
  right_avg_y_ = right_avg_y;
  right_length_ = right_length;
  right_available_ = right_available;
}

// -------------------------------------------------------------------------------------------------
void HistoryEntry::passDataOn(){
// -------------------------------------------------------------------------------------------------
  mid_prev_x_ = mid_x_;
  mid_prev_slope_ = mid_slope_;
  mid_prev_available_ = mid_available_;
  mid_x_ = mid_next_x_;
  mid_slope_ = mid_next_slope_;
  mid_available_ = mid_next_available_;
  mid_next_x_ = 0;
  mid_next_slope_ = 0;
  mid_next_available_ = false;
}

