#include "GridSearch.h"

#include "CompareSearchNode.h"
#include <queue>
#include <iostream>

// -------------------------------------------------------------------------------------------------
void GridSearch::addMap(SearchMap const & map) {
// -------------------------------------------------------------------------------------------------
  maps_.push_back(&map);
}

// -------------------------------------------------------------------------------------------------
std::vector<Vector2> GridSearch::getPlan(Vector2 starting_position, int length, int & cost) {
// -------------------------------------------------------------------------------------------------
  std::list<SearchNode*> delete_list;
  SearchNode * start = new SearchNode(starting_position, getCost(starting_position), 1, NULL, 4);
  delete_list.push_back(start);

  std::priority_queue<SearchNode*, std::vector<SearchNode*>, CompareSearchNode> queue;
  queue.push(start);
  
  size_t ctr = 0;

  while(true) {
    // Retrieve the front element
    SearchNode * cur = queue.top();

    if (cur->get_path_length() >= length) break;

    // Expand the nodes
    //double cost_factor = 1.0 / (cur->get_path_length() + 1);
    double cost_factor = 1.0;
    SearchNode *expanded_search_node;
    std::list<SearchNode*> expanded_nodes;
    for (int i = 0; i < action_set_.size(); i++) {
      if (getActionSign(i) != 0 && getActionSign(cur->get_action()) != 0 && getActionSign(i) != getActionSign(cur->get_action())) {
        //std::cout << "Angle: " << action_set_[i].angle() << " and current angle: " << action_set_[cur->get_action()].angle() << ", signs are: old = "
        //  << getActionSign(cur->get_action()) << " new = " << getActionSign(i) << std::endl;
        continue;
      }
      Vector2 coordinates(cur->get_coordinates().get_x() + action_set_[i].x_diff(), cur->get_coordinates().get_y() + action_set_[i].y_diff());
      int constant_cost = action_set_[i].constant_cost_diff();
      int local_cost = action_set_[i].linear_cost_diff() * cost_factor * getCost(coordinates);
      int parent_cost = cur->get_cost();
      int direction_cost = action_set_[i].compare(action_set_[cur->get_action()]) * 100;
      int node_cost = direction_cost + parent_cost + local_cost + constant_cost;
      
      ctr++;
      
      expanded_search_node = new SearchNode(coordinates, node_cost, cur->get_path_length() + 1, cur, i);
      delete_list.push_back(expanded_search_node);
      expanded_nodes.push_back(expanded_search_node);
    }
    
    queue.pop();
    for (std::list<SearchNode*>::iterator it = expanded_nodes.begin(); it != expanded_nodes.end(); ++it) {
      queue.push(*it);
    }
  }

  // Retrieve the min element
  SearchNode * min = queue.top();

  std::vector<Vector2> result;
  result.push_back(min->get_coordinates());

  SearchNode * parent = min->get_parent();
  while (parent != NULL) {
    result.push_back(parent->get_coordinates());
    parent = parent->get_parent();
  }

  cost = min->get_cost();

  while (delete_list.size() != 0) {
    delete delete_list.back();
    delete_list.pop_back();
  }
  
  /*for (int i = result.size() - 1; i >= 1; i--) {
    std::cout << "(" << result[i].get_x() << ", " << result[i].get_y() << ") - ";
  }
  std::cout << "(" << result[0].get_x() << ", " << result[0].get_y() << ")" << std::endl;*/
  
  return result;
}

// -------------------------------------------------------------------------------------------------
int GridSearch::getCost(Vector2 position) const {
// -------------------------------------------------------------------------------------------------
  double costs = 0;

  // Accumulate cost based on each map's occupancies
  for (std::list<SearchMap const *>::const_iterator it = maps_.begin(); it != maps_.end(); it++) {
    costs += (*it)->get_cost(position);
  }

  return (int)costs;
}

// -------------------------------------------------------------------------------------------------
int GridSearch::getActionSign(int index) {
// -------------------------------------------------------------------------------------------------
  if (action_set_[index].angle() == -5) return 0;
  if (action_set_[index].angle() > -5) return 1;
  if (action_set_[index].angle() < -5) return -1;
  return 0;
}
