#pragma once

#include <memory>
#include "../input/input.hpp"
#include "../types/pid_gains.hpp"

namespace robotics::filter {

template <typename T>
class PID : public input::Input<PIDGains> {
 private:
  T integral_;
  T prev_error_;

  input::IInputController<PIDGains> *controller_ = nullptr;

 public:
  PID(T kP, T kI, T kD) : integral_(0), prev_error_(0) {
    this->controller_ = this->GetController();
    this->controller_->SetValue({kP, kI, kD});
  }

  T Update(T target, T feedback, T dt) {
    PIDGains value = controller_->GetValue();

    T error = target - feedback;
    integral_ += error * dt;
    T derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return value.p * error + value.i * integral_ + value.d * derivative;
  }
};
}  // namespace robotics::filter