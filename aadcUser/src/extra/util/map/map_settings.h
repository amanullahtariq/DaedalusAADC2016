#ifndef __MAP_SETTINGS_H__
#define __MAP_SETTINGS_H__

// Map size in cm
static const int REAL_MAP_HEIGHT = 300;//500;//360, 500;
static const int REAL_MAP_WIDTH = 300;//180, 300;

static const int CAR_WIDTH = 21;
static const int CAR_LENGTH = 21;

static const int CELL_SIZE = 3;
static const size_t HISTORY_SIZE = 2;
static const float DISCOUNT_FACTOR = 0;

static const int MAP_HEIGHT = REAL_MAP_HEIGHT/CELL_SIZE;
static const int MAP_WIDTH = REAL_MAP_WIDTH/CELL_SIZE;

static const int CAR_ROW = MAP_HEIGHT - 50/CELL_SIZE;
static const int CAR_COL = MAP_WIDTH/2;

static const double INITIAL_OCCUPANCY = 0.05;

#endif // __MAP_SETTINGS_H__
