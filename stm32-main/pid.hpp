#pragma once

template <typename T>
class PID {
  T kP_, kI_, kD_;
  T integral_;
  T prev_error_;

 public:
  PID(T kP, T kI, T kD)
      : kP_(kP), kI_(kI), kD_(kD), integral_(0), prev_error_(0) {}

  T Update(T target, T feedback, T dt) {
    T error = target - feedback;
    integral_ += error * dt;
    T derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kP_ * error + kI_ * integral_ + kD_ * derivative;
  }
};