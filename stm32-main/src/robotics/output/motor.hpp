#pragma once

#include "output.hpp"

namespace robotics::output {
template <typename T>
class Motor : public Output<T> {
 private:
  /**
   * @brief Set the Speed
   * @param speed in the range [-1, 1]
   */
  virtual void SetSpeed(T speed) = 0;

  void Update(T value) override { SetSpeed(value); }

 public:
};
}  // namespace robotics::output