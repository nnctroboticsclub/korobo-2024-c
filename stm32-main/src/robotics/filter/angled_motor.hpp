#pragma once

#include "pid.hpp"
#include "../node/node.hpp"

#include "./angle_smoother.hpp"

namespace robotics::filter {
template <typename T>
class AngledMotor {
 public:
  filter::PID<float> pid{1.0f / 360, 2.0f, 0.52f, 0.13f};

  Node<float> feedback;
  Node<float> goal;
  Node<float> output;

 private:
  AngleNormalizer<float> feedback_normalizer;
  AngleNormalizer<float> goal_normalizer;

 public:
  AngledMotor() : feedback(0), goal(0), output(0) {
    feedback.Link(feedback_normalizer.input);
    feedback_normalizer.output.Link(pid.fb_);

    goal.Link(goal_normalizer.input);
    goal_normalizer.output.Link(pid.goal_);

    pid.output_.Link(output);
  }

  void Update(float dt) { pid.Update(dt); }

  void Reset() {
    feedback_normalizer.Reset();
    goal_normalizer.Reset();
  }
};
}  // namespace robotics::filter