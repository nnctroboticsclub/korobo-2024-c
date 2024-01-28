#pragma once

#include "pid.hpp"
#include "../node/node.hpp"

namespace robotics::filter {
template <typename T>
class AngledMotor {
 public:
  filter::PID<float> pid{3.0f, 0.0f, 0.0f};

  Node<float> feedback;
  Node<float> goal;
  Node<float> output;

 public:
  AngledMotor() : feedback(0), goal(0), output(0) {
    feedback.Link(pid.fb_);
    goal.Link(pid.goal_);

    pid.output_.Link(output);
  }

  void Update(float dt) { pid.Update(dt); }
};
}  // namespace robotics::filter