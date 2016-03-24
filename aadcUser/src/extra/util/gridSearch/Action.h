#ifndef __ACTION_H_
#define __ACTION_H_

class Action {
  public:
    Action();
    Action(int x_diff, int y_diff, int constant_cost_diff, float linear_cost_diff, int angle, int speed);
    int x_diff() const;
    int y_diff() const;
    int constant_cost_diff() const;
    int linear_cost_diff() const;
    int angle() const;
    int speed() const;
    float compare(Action const & other);
    
  private:
    int x_diff_;
    int y_diff_;
    int constant_cost_diff_;
    float linear_cost_diff_;
    int angle_;
    int speed_;
};

#endif // __ACTION_H_
