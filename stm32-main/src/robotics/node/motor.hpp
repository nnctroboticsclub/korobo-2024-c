#pragma once

#include "node.hpp"

namespace robotics::node {
template <typename T>
class Motor : public Node<T> {
 private:
  /**
   * @brief Set the Speed
   * @param speed in the range [-1, 1]
   */
  virtual void SetSpeed(T speed) = 0;

 public:
  Motor() {
    this->SetChangeCallback([this](T value) { this->SetSpeed(value); });
  }
};
}  // namespace robotics::node