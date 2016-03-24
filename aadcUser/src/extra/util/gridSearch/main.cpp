#include <iostream>

#include "TestMap.h"
#include "GridSearch.h"
#include <vector>
#include <utility>
#include "SearchNode.h"

int main() {
    float test_array_1[][11] = {
        {0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0}
    };

    float test_array_2[][11] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    float** map_array_1 = new float*[5];
    float** map_array_2 = new float*[5];
    for (int row = 0; row < 5; row++) {
        map_array_1[row] = new float[11];
        map_array_2[row] = new float[11];

        for (int col = 0; col < 11; col++) {
            map_array_1[row][col] = test_array_1[row][col];
            map_array_2[row][col] = test_array_2[row][col];
        }
    }

    TestMap t1(map_array_1, 100, 5, 11, 6);
    TestMap t2(map_array_2, 1000, 5, 11, 6);

    GridSearch s;
    s.addMap(t1);
    s.addMap(t2);

    std::pair<int, int> start(5, 4);
    s.getPlan(start, 3);

    for (int row = 0; row < 5; row++) {
        delete[] map_array_1[row];
        delete[] map_array_2[row];
    }
    delete[] map_array_1;
    delete[] map_array_2;
}