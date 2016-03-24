#ifndef __ACTION_SET_H_
#define __ACTION_SET_H_

#include "Action.h"

class ActionSet {
  public:
    ActionSet();
    ~ActionSet();
    Action operator[](int index);
    int size();
    
  protected:
    Action *action_set_;
    int size_;
};

#endif // __ACTION_SET_H_
