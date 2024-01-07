#pragma once

namespace robotics::output {
template <typename T>
class Motor {
 public:
  // Set the speed of the motor.
  // The value of `speed` is in the range of [-1, 1].
  virtual void SetSpeed(T speed) = 0;
};
}  // namespace robotics::output