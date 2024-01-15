#pragma once

#include "pid.hpp"
#include "../node/node.hpp"

namespace robotics::filter {
template <typename T>
class AngledMotor {
 public:
  filter::PID<float> pid;

  Node<float> feedback;
  Node<float> goal;
  Node<float> output;

 public:
  AngledMotor() : pid(0, 0, 0), feedback(0), goal(0), output(0) {
    feedback.Link(pid.fb_);
    goal.Link(pid.goal_);

    pid.output_.Link(output);
  }

  void Update(float dt) { pid.Update(dt); }
};
}  // namespace robotics::filter