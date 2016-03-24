#include "a_star_action_set.h"

// -------------------------------------------------------------------------------------------------
AStarActionSet::AStarActionSet(int index) {
// -------------------------------------------------------------------------------------------------
  // Top Left
  if (index == 0) {
    Action action_set_left[] = {
      Action(-1, 0, 1, 1, -99, -99), // Move top
      Action(0, -1, 1, 1, -99, -99) // Move left     
    };
    size_ = sizeof(action_set_left)/sizeof(Action);
    action_set_ = new Action[size_];
    for (int i = 0; i < size(); i++) action_set_[i] = action_set_left[i];
  }
  // Straight
  else if (index == 1) {
    Action action_set_straight[] = {
      Action(-1, 0, 1, 1, -99, -99), // Move top
      Action(0, -1, 1, 1, -99, -99), // Move left
      Action(0, 1, 1, 1, -99, -99) // Move right
    };
    size_ = sizeof(action_set_straight)/sizeof(Action);
    action_set_ = new Action[size_];
    for (int i = 0; i < size(); i++) action_set_[i] = action_set_straight[i];
  }
  // Top Right
  else if (index == 2) {
    Action action_set_right[] = {
      Action(-1, 0, 1, 1, -99, -99), // Move top
      Action(0, 1, 1, 1, -99, -99)  // Move right
    };
    size_ = sizeof(action_set_right)/sizeof(Action);
    action_set_ = new Action[size_];
    for (int i = 0; i < size(); i++) action_set_[i] = action_set_right[i];
  }
}
