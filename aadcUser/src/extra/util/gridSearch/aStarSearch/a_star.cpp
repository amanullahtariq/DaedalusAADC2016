#include "a_star.h"
#include <queue>
#include <list>
#include "../CompareSearchNode.h"
#include <iostream>

// -------------------------------------------------------------------------------------------------
AStar::AStar(SearchMap const & map) {
// -------------------------------------------------------------------------------------------------
  map_ = &map;
}

// -------------------------------------------------------------------------------------------------
int AStar::getPlanCost(Vector2 start, std::vector<Vector2> const & end, AStarActionSet action_set, int type, std::vector<Vector2> & path) {
// -------------------------------------------------------------------------------------------------
  //std::cout << "In AStar" << std::endl;
  AStarSearchNode * start_node = new AStarSearchNode(start, getHeuristicCost(start, end), 0, 1, NULL, -1);
  std::priority_queue<AStarSearchNode*, std::vector<AStarSearchNode*>, CompareSearchNode> queue;
  queue.push(start_node);
  std::vector<AStarSearchNode*> closed;
  std::vector<AStarSearchNode*> delete_list;
  delete_list.push_back(start_node);
  
  while(queue.size() > 0) {
    AStarSearchNode * current_node = queue.top();
    bool found = false;
    bool update_cost = false;
    for (int i = 0; i < closed.size(); i++) {
      if (closed[i]->get_coordinates() == current_node->get_coordinates()) {
        found = true;
        if (closed[i]->get_path_cost() > current_node->get_path_cost()) {
          closed[i]->update_path_cost(current_node->get_path_cost());
          update_cost = true;
        }
      }
    }
    //if(found) std::cout << "Duplicate detected: " << current_node->get_coordinates().get_x() << ", " << current_node->get_coordinates().get_y() << std::endl;
    //if(update_cost) std::cout << "Updated cost" << std::endl;
    //if(!found) std::cout << "Looking at new point" << current_node->get_coordinates().get_x() << ", " << current_node->get_coordinates().get_y() << std::endl;
    
    if (!found) closed.push_back(current_node);
    
    /*for (int i = 0; i < closed.size(); i++) {
      std::cout << "(" << closed[i]->get_coordinates().get_x() << ", " << closed[i]->get_coordinates().get_y() << ")" << std::endl;
    }*/
    
    if (!found || update_cost){
      if (isGoal(current_node, end)) {
        //std::cout << "Found goal" << std::endl;
        int result = current_node->get_path_cost();
        
        //std::cout << "PATH COSTS" << std::endl;
        AStarSearchNode * current = current_node;
        while(current != NULL) {
          //std::cout << "(" << current->get_coordinates().get_x() << ", " << current->get_coordinates().get_y() << ") " << current->get_cost() << " (" << current->get_heuristic() << ", " << current->get_path_cost() << ")" << std::endl;
          path.push_back(current->get_coordinates());
          current = (AStarSearchNode *) current->get_parent();
        }
        while (delete_list.size() > 0) {
          delete delete_list.back();
          delete_list.pop_back();
        }
        return result;
      }
      /*std::cout << "Goal" << end.get_x() << ", " << end.get_y() << std::endl;
      std::cout << "Start expanding node with coordinates " << current_node->get_coordinates().get_x() << ", " 
        << current_node->get_coordinates().get_y() << " with cost " << current_node->get_cost() 
        << " with vals " << current_node->get_heuristic() << " and " << current_node->get_path_cost() << std::endl;*/
      
      std::list<AStarSearchNode*> tmp;
      for (int i = 0; i < action_set.size(); i++) {
        Vector2 coordinates(current_node->get_coordinates().get_x() + action_set[i].x_diff(), current_node->get_coordinates().get_y() + action_set[i].y_diff());
        if (!checkBoundaries(coordinates, end, type)) continue;
        int heuristic_cost = getHeuristicCost(coordinates, end);
        int path_cost = getPathCost(current_node) + action_set[i].constant_cost_diff();
        
        AStarSearchNode * expanded_search_node = new AStarSearchNode(coordinates, heuristic_cost, path_cost, current_node->get_path_length() + 1, current_node, i);
        delete_list.push_back(expanded_search_node);
        tmp.push_back(expanded_search_node);
      }

      //delete current_node;
      queue.pop();
      //std::cout << "--------------" << std::endl;
      //std::cout << "Current costs: " << current_node->get_coordinates().get_x() << ", " << current_node->get_coordinates().get_y() << " -> " << current_node->get_cost() 
          //<< " with vals " << current_node->get_heuristic() << " and " << current_node->get_path_cost() << std::endl;
      while(tmp.size() > 0) {
        queue.push(tmp.front());
        //std::cout << "   Successor costs: " << tmp.front()->get_coordinates().get_x() << ", " << tmp.front()->get_coordinates().get_y() << " -> " << tmp.front()->get_cost() 
          //<< " with vals " << tmp.front()->get_heuristic() << " and " << tmp.front()->get_path_cost() << std::endl;
        tmp.pop_front();
      }
    } else {
      //delete current_node;
      queue.pop();
    }
  }
  while (delete_list.size() > 0) {
    delete delete_list.back();
    delete_list.pop_back();
  }
  //std::cout << "Before return" << std::endl;
  return -1;
}
 
// -------------------------------------------------------------------------------------------------
int AStar::getHeuristicCost(Vector2 current_position, std::vector<Vector2> const & goal_position) {
// -------------------------------------------------------------------------------------------------
  int result = (abs(current_position.get_x() - goal_position[0].get_x()) + abs(current_position.get_y() - goal_position[0].get_y()));
  for (size_t i = 1; i < goal_position.size(); i++) {
    int current = (abs(current_position.get_x() - goal_position[i].get_x()) + abs(current_position.get_y() - goal_position[i].get_y()));
    if (current < result) result = current;
  }
  return result; 
}

// -------------------------------------------------------------------------------------------------
int AStar::getPathCost(AStarSearchNode * current_node) {
// -------------------------------------------------------------------------------------------------
  return current_node->get_path_cost() + map_->get_cost(Vector2(current_node->get_coordinates().get_y(), current_node->get_coordinates().get_x()), WIDTH, HEIGHT);
}

// -------------------------------------------------------------------------------------------------
bool AStar::isGoal(AStarSearchNode * current_node, std::vector<Vector2> const & goal_position) {
// -------------------------------------------------------------------------------------------------
  return getHeuristicCost(current_node->get_coordinates(), goal_position) == 0;
}

// -------------------------------------------------------------------------------------------------
bool AStar::checkBoundaries(Vector2 coordinates, std::vector<Vector2> const & end, int type) {
// -------------------------------------------------------------------------------------------------
  switch(type) {
    case 0:
      return !(coordinates.get_x() < end[0].get_x() || coordinates.get_y() < end[0].get_y());
    case 1:
      return !(coordinates.get_y() < end[0].get_y() - 3 || coordinates.get_y() > end[end.size() - 1].get_y() + 3 || coordinates.get_x() < end[0].get_x());
    case 2:
      return !(coordinates.get_x() < end[0].get_x() || coordinates.get_y() > end[0].get_y());
  }
  return true;
}
