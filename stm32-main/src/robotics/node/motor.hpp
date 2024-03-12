#pragma once

#include "node.hpp"

namespace robotics::node {
template <typename T>
class Motor : public Node<T> {
 public:
  Node<T> factor;

 private:
  /**
   * @brief Set the Speed
   * @param speed in the range [-1, 1]
   */
  virtual void SetSpeed(T speed) = 0;

  void Update() {
    auto factor_value = factor.GetValue();
    auto speed = this->GetValue();
    auto effective_speed = factor_value * speed;

    SetSpeed(effective_speed);
  }

 public:
  Motor() {
    factor.SetValue(1);

    this->SetChangeCallback([this](T value) { this->Update(); });
    factor.SetChangeCallback([this](T value) { this->Update(); });
  }
};
}  // namespace robotics::node