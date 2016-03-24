#include "history.h"
#include <assert.h>

// -------------------------------------------------------------------------------------------------
bool History::insert(HistoryEntry new_entry) {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > history_size_){
    entries_.pop_back();
    entries_.insert(entries_.begin(), new_entry);
  } else entries_.insert(entries_.begin(), new_entry);
  return true;
}

// -------------------------------------------------------------------------------------------------
int History::left_get_weighted_x() {
// -------------------------------------------------------------------------------------------------
  double weighted_x = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).left_available_){
       weighted_x += entries_.at(i).left_x_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (int)(weighted_x/devisor);
}

// -------------------------------------------------------------------------------------------------
double History::left_get_weighted_slope() {
// -------------------------------------------------------------------------------------------------
  double weighted_slope = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).left_available_){
       weighted_slope += entries_.at(i).left_slope_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (weighted_slope/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
double History::left_get_length() {
// -------------------------------------------------------------------------------------------------
  double length = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).left_available_){
       length += entries_.at(i).left_length_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (length/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
int History::mid_get_weighted_x() {
// -------------------------------------------------------------------------------------------------
  double weighted_x = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_available_){
       weighted_x += entries_.at(i).mid_x_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (int)(weighted_x/devisor);
}

// -------------------------------------------------------------------------------------------------
double History::mid_get_weighted_slope() {
// -------------------------------------------------------------------------------------------------
  double weighted_slope = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_available_){
       weighted_slope += entries_.at(i).mid_slope_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (weighted_slope/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
double History::mid_get_length() {
// -------------------------------------------------------------------------------------------------
  double length = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_available_){
       length += entries_.at(i).mid_length_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (length/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
int History::mid_next_get_weighted_x() {
// -------------------------------------------------------------------------------------------------
  double weighted_x = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_next_available_){
       weighted_x += entries_.at(i).mid_next_x_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (int)(weighted_x/devisor);
}

// -------------------------------------------------------------------------------------------------
double History::mid_next_get_weighted_slope() {
// -------------------------------------------------------------------------------------------------
  double weighted_slope = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_next_available_){
       weighted_slope += entries_.at(i).mid_next_slope_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (weighted_slope/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
int History::mid_prev_get_weighted_x() {
// -------------------------------------------------------------------------------------------------
  double weighted_x = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_prev_available_){
       weighted_x += entries_.at(i).mid_prev_x_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (int)(weighted_x/devisor);
}

// -------------------------------------------------------------------------------------------------
double History::mid_prev_get_weighted_slope() {
// -------------------------------------------------------------------------------------------------
  double weighted_slope = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).mid_prev_available_){
       weighted_slope += entries_.at(i).mid_prev_slope_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (weighted_slope/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
int History::right_get_weighted_x() {
// -------------------------------------------------------------------------------------------------
  double weighted_x = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).right_available_){
       weighted_x += entries_.at(i).right_x_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (int)(weighted_x/devisor);
}

// -------------------------------------------------------------------------------------------------
double History::right_get_weighted_slope() {
// -------------------------------------------------------------------------------------------------
  double weighted_slope = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).right_available_){
       weighted_slope += entries_.at(i).right_slope_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (weighted_slope/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
double History::right_get_length() {
// -------------------------------------------------------------------------------------------------
  double length = 0;
  int devisor = 0;
  for(size_t i = 0; i < entries_.size(); i++){
     if(entries_.at(i).right_available_){
       length += entries_.at(i).right_length_ * (double)(entries_.size() - i);
       devisor += (entries_.size() - i);
     }   
  }
  return (length/(double)(devisor));
}

// -------------------------------------------------------------------------------------------------
bool History::left_empty() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++) if(entries_.at(i).left_available_) return false;   
  return true;
}

// -------------------------------------------------------------------------------------------------
bool History::mid_empty() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++) if(entries_.at(i).mid_available_) return false;   
  return true;
}

// -------------------------------------------------------------------------------------------------
bool History::mid_next_empty() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++) if(entries_.at(i).mid_next_available_) return false;   
  return true;
}

// -------------------------------------------------------------------------------------------------
bool History::mid_prev_empty() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++) if(entries_.at(i).mid_prev_available_) return false;   
  return true;
}

// -------------------------------------------------------------------------------------------------
bool History::right_empty() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++) if(entries_.at(i).right_available_) return false;   
  return true;
}

// -------------------------------------------------------------------------------------------------
int History::get_size() {
// -------------------------------------------------------------------------------------------------
  return entries_.size();
}

// -------------------------------------------------------------------------------------------------
bool History::jumpSegment() {
// -------------------------------------------------------------------------------------------------
  for(size_t i = 0; i < entries_.size(); i++){
    entries_.at(i).passDataOn();
  }
  return true;
}

// -------------------------------------------------------------------------------------------------
int History::get_left_x() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].left_avg_x_;
  return 0;
}

// -------------------------------------------------------------------------------------------------
int History::get_left_y() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].left_avg_y_;
  return 0;
}

// -------------------------------------------------------------------------------------------------
int History::get_mid_x() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].mid_avg_x_;
  return 0;
}

// -------------------------------------------------------------------------------------------------
int History::get_mid_y() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].mid_avg_y_;
  return 0;
}

// -------------------------------------------------------------------------------------------------
int History::get_right_x() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].right_avg_x_;
  return 0;
}

// -------------------------------------------------------------------------------------------------
int History::get_right_y() {
// -------------------------------------------------------------------------------------------------
  if(entries_.size() > 0) return entries_[0].right_avg_y_;
  return 0;
}

