#include "cell.h"

#include <iostream>

// -------------------------------------------------------------------------------------------------
Cell::Cell(size_t history_size, float discount_factor) {
// -------------------------------------------------------------------------------------------------
	history_size_ = history_size;
	occupancy_history_ = new float[history_size_];
	index_ = 0;
	filled_ = false;
	discount_factor_ = discount_factor;
	float current_discount_factor = 1;
	norm_ = 0;
	
	for (size_t i = 0; i < history_size_; i++) {
		norm_ += current_discount_factor;
		current_discount_factor *= discount_factor_;
		occupancy_history_[i] = INITIAL_OCCUPANCY;
	}
}

// -------------------------------------------------------------------------------------------------
Cell::~Cell() {
// -------------------------------------------------------------------------------------------------
	delete[] occupancy_history_;
	occupancy_history_ = NULL;
}

// -------------------------------------------------------------------------------------------------
void Cell::update(float occupancy_value) {
// -------------------------------------------------------------------------------------------------
	if (occupancy_value != occupancy_value) {
	  return;
	}
	
	occupancy_history_[index_++] = occupancy_value;
	
	if (index_ >= history_size_) { 
		index_ = 0;
		filled_ = true;
	}
}

// -------------------------------------------------------------------------------------------------
float Cell::occupancy() {
// -------------------------------------------------------------------------------------------------
	size_t current_index = 0;
	if (index_ == 0) {
	  if (!filled_) return INITIAL_OCCUPANCY;
	  else current_index = history_size_ - 1;
  } else current_index = index_ - 1;
	
	size_t target_index = index_;
	if (!filled_) target_index = 0;
	
	float current_discount_factor = 1;
	float occupancy = 0;
	
	while(current_index != target_index) {
	  float delta_occupancy = current_discount_factor * occupancy_history_[current_index];

		occupancy += delta_occupancy;
		current_discount_factor *= discount_factor_;
		if (current_index > 0) current_index--;
		else current_index = history_size_ - 1;
	}

	float result = occupancy/norm_;
	return result;
}

// -------------------------------------------------------------------------------------------------
size_t Cell::history_size() {
// -------------------------------------------------------------------------------------------------
  return history_size_;
}
