#pragma once

#include <memory>
#include "../node/node.hpp"
#include "../types/pid_gains.hpp"

namespace robotics::filter {

template <typename T>
class PID {
 private:
  T integral_;
  T prev_error_;

 public:
  Node<PIDGains> gains;

  Node<T> fb_;
  Node<T> goal_;
  Node<T> output_;

 public:
  PID(T kP = 0, T kI = 0, T kD = 0) : integral_(0), prev_error_(0) {
    gains.SetValue(PIDGains(kP, kI, kD));
  }

  void Update(T dt) {
    PIDGains value = gains.GetValue();

    T target = goal_.GetValue();
    T feedback = fb_.GetValue();

    T error = target - feedback;
    integral_ += error * dt;
    T derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    T output = value.p * error + value.i * integral_ + value.d * derivative;
    output_.SetValue(output);
  }
};
}  // namespace robotics::filter