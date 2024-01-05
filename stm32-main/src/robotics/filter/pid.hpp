#pragma once

#include <memory>

namespace robotics::filter {

class IPIDController {
 public:
  virtual void UpdateKp(float kp) = 0;
  virtual void UpdateKi(float ki) = 0;
  virtual void UpdateKd(float kd) = 0;
};

template <typename T>
class PID {
  class Controller : public IPIDController {
    PID<T>* pid_;

   public:
    Controller(PID<T>* pid) : pid_(pid) {}

    void UpdateKp(float kp) override { pid_->kP_ = kp; }
    void UpdateKi(float ki) override { pid_->kI_ = ki; }
    void UpdateKd(float kd) override { pid_->kD_ = kd; }
  };

 private:
  T kP_, kI_, kD_;
  T integral_;
  T prev_error_;

  std::shared_ptr<Controller> controller_;

 public:
  PID(T kP, T kI, T kD)
      : kP_(kP), kI_(kI), kD_(kD), integral_(0), prev_error_(0) {
    controller_ = std::make_shared<Controller>(this);
  }

  T Update(T target, T feedback, T dt) {
    T error = target - feedback;
    integral_ += error * dt;
    T derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kP_ * error + kI_ * integral_ + kD_ * derivative;
  }

  std::shared_ptr<Controller> GetController() { return controller_; }
};
}  // namespace robotics::filter