#ifndef __COMPARE_SEARCH_NODE_H__
#define __COMPARE_SEARCH_NODE_H__ 

#include "SearchNode.h"

class CompareSearchNode {
  public:
    bool operator()(SearchNode * a, SearchNode * b) {
        if (a->get_cost() < b->get_cost()) return false;
        if (a->get_cost() == b->get_cost()) {
          if (a->get_path_length() <= b->get_path_length()) return false;
        }
        return true;
    };
};

#endif
