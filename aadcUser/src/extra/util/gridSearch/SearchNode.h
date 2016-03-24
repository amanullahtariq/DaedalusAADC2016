#ifndef __SEARCH_NODE_H__
#define __SEARCH_NODE_H__

#include "../vector2.h"

class SearchNode {
  public:
    SearchNode(Vector2 coordinates, int cost, int path_length, SearchNode * parent, int action);

    Vector2 get_coordinates() const;
    int get_cost() const;
    int get_path_length() const;
    SearchNode * get_parent() const;
    int get_action() const;

  protected:
    Vector2 coordinates_;
    int cost_;
    int path_length_;
    SearchNode * parent_;
    int action_;
};

#endif  // __SEARCH_NODE_H__
