#pragma once

#include "packet.hpp"
#include "../robotics/input/input.hpp"

namespace controller {

template <typename T>
struct InputBase {
 private:
  bool Filter(RawPacket const &packet) virtual = 0;

  void Parse(RawPacket const &packet) virtual = 0;

 public:
  int assigned_id_;
  T value;
  robotics::input::IInputController<T> *controller = nullptr;

  InputBase(int id) : assigned_id_(id) {}

  bool Parse(RawPacket const &packet) {
    if (!this->Filter(packet)) {
      return false;
    }

    this->Parse(packet);

    if (controller != nullptr) {
      controller->SetValue({x, y});
    }

    return true;
  }

  void Connect(robotics::input::IInputController<T> *controller) {
    this->controller = controller;
  }
};

}  // namespace controller
