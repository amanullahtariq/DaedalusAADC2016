#include "a_star_search_node.h"

// -------------------------------------------------------------------------------------------------
AStarSearchNode::AStarSearchNode(Vector2 coordinates, int heuristic, int path_cost, int path_length, SearchNode * parent, int action) 
  : SearchNode(coordinates, (heuristic+path_cost), path_length, parent, action) {
// -------------------------------------------------------------------------------------------------
  heuristic_ = heuristic;
  path_cost_ = path_cost;
}

// -------------------------------------------------------------------------------------------------
int AStarSearchNode::get_heuristic() {
// -------------------------------------------------------------------------------------------------
  return heuristic_;
}

// -------------------------------------------------------------------------------------------------
int AStarSearchNode::get_path_cost() {
// -------------------------------------------------------------------------------------------------
  return path_cost_;
}

// -------------------------------------------------------------------------------------------------
void AStarSearchNode::update_path_cost(int path_cost) {
// -------------------------------------------------------------------------------------------------
  path_cost_ = path_cost;
  cost_ = path_cost_ + heuristic_;
}
