#ifndef __A_STAR_SEARCH_NODE_H__
#define __A_STAR_SEARCH_NODE_H__

#include "../SearchNode.h"

class AStarSearchNode : public SearchNode {
  public:
    AStarSearchNode(Vector2 coordinates, int heuristic, int path_cost, int path_length, SearchNode * parent, int action);
    int get_heuristic();
    int get_path_cost();
    void update_path_cost(int path_cost);

  private:
    int heuristic_;
    int path_cost_;
};

#endif // __A_STAR_SEARCH_NODE_H__
