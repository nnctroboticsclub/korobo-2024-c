#pragma once

#include "packet.hpp"
#include "../robotics/input/input.hpp"

namespace controller {

template <typename T>
struct ControllerBase {
 public:
  virtual bool Filter(RawPacket const &packet) = 0;

  virtual void Parse(RawPacket const &packet) = 0;

 public:
  int assigned_id_;
  T value;
  robotics::input::IInputController<T> *controller = nullptr;

  ControllerBase(int id) : assigned_id_(id), value() {}

  bool Pass(RawPacket const &packet) {
    if (!this->Filter(packet)) {
      return false;
    }

    this->Parse(packet);

    if (controller != nullptr) {
      controller->SetValue(value);
    }

    return true;
  }

  void Connect(robotics::input::IInputController<T> *controller) {
    this->controller = controller;
  }
};

}  // namespace controller
