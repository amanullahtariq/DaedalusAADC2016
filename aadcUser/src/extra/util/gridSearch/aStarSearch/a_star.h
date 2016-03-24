#ifndef __A_STAR_H__
#define __A_STAR_H__

#include "a_star_search_node.h"
#include "../SearchMap.h"
#include "../../vector2.h"
#include "a_star_action_set.h"
#include <vector>
 
class AStar {
  public:
    AStar(SearchMap const & map);
    int getPlanCost(Vector2 start, std::vector<Vector2> const & end, AStarActionSet action_set, int type, std::vector<Vector2> & path);
    int getHeuristicCost(Vector2 current_position, std::vector<Vector2> const & goal_position);
    int getPathCost(AStarSearchNode * current_node);
    bool checkBoundaries(Vector2 coordinates, std::vector<Vector2> const & end, int type);
    bool isGoal(AStarSearchNode * current_node, std::vector<Vector2> const & goal_position);
    
  private:
    SearchMap const * map_;
    static const int HEIGHT = 9;
    static const int WIDTH = 9;
};

#endif // __A_STAR_H__
