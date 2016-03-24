#include "TestMap.h"

TestMap::TestMap(float** map_array, int cost, size_t cell_size, int map_width, int map_height)
    : SearchMap(cost, cell_size, map_width, map_height) {
    map_array_ = map_array;
}

float TestMap::isOccupied(std::pair<int, int> position) const {
    return map_array_[position.second][position.first];
}