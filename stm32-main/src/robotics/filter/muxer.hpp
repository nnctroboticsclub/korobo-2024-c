#pragma once

#include <array>

namespace robotics::filter {
template <typename T, int N>
struct Muxer {
  std::array<Node<T>, N> inputs_;
  Node<T> output_;
  int selected_ = 0;

  Muxer() {
    int i = 0;
    for (auto& input : inputs_) {
      input.SetChangeCallback([this, i](T value) {
        if (i == selected_) {
          output_.SetValue(value);
        }
      });
      i++;
    }
  }

  void Select(int index) {
    if (index < 0 || index >= N) {
      return;
    }
    selected_ = index;
    output_.SetValue(inputs_[index].GetValue());
  }
};
}  // namespace robotics::filter