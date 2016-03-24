#ifndef _CELL_H_
#define _CELL_H_

#include <stdio.h>
#include "map_settings.h"

class Cell {
 public:
 	Cell(size_t history_size, float discount_factor);
 	~Cell();
 	float occupancy();
 	void update(float occupancy_value);
 	size_t history_size();

 private:
 	float *occupancy_history_;
 	size_t history_size_;
 	size_t index_;
 	bool filled_;
 	float discount_factor_;
 	float norm_;
};

#endif // _CELL_H_
