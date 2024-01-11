#pragma once

#include "pid.hpp"
#include "../input/input.hpp"
#include "../output/output.hpp"

namespace robotics::filter {
template <typename T>
class AngledMotor {
 public:
  filter::PID<float> pid;

  input::Input<float> feedback;
  input::Input<float> goal;
  output::Output<float> output;

 public:
  AngledMotor() {}

  input::IInputController<PIDGains>* GetPIDController() {
    return angle_.GetController();
  }

  void Update(float dt) {
    float angle = angle_.GetValue();
    float goal = goal_.GetValue();
    float feedback = feedback_.GetValue();

    float power = pid.Update(goal, feedback, dt);
    output.SetOutputValue(power);
  }
};
}  // namespace robotics::filter