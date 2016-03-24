#ifndef __GRID_SEARCH_H__
#define __GRID_SEARCH_H__

#include "SearchNode.h"
#include "SearchMap.h"
#include "../vector2.h"
#include "ActionSet.h"

class GridSearch {
  public:
    enum Actions {UL, U, UR};

    void addMap(SearchMap const & map);
    std::vector<Vector2> getPlan(Vector2 starting_position, int length, int & cost);

  private:
    int getCost(Vector2 position) const;
    int getActionSign(int index);
    std::list<SearchMap const *> maps_;
    ActionSet action_set_;
};

#endif  // __GRID_SEARCH_H__
