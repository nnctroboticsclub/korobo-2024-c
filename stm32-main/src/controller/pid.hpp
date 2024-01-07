#pragma once

#include "packet.hpp"
#include "../robotics/filter/pid.hpp"

namespace controller {

class PID {
  using PIDController = robotics::filter::IPIDController;
  using PIDControllerPtr = std::shared_ptr<PIDController>;

  int assigned_id_;
  PIDControllerPtr controller;

 public:
  PID(int id) : assigned_id_(id), controller(nullptr) {}

  void ConnectGain(PIDControllerPtr controller) {
    this->controller = controller;
  }

  bool Parse(RawPacket const& packet) {
    if (packet.element_id != (0xA0 | assigned_id_)) {
      return false;
    }

    if (controller == nullptr) {
      return false;
    }

    float p = packet.data[0] / 255.0f * 10.0f;
    float i = packet.data[1] / 255.0f * 10.0f;
    float d = packet.data[2] / 255.0f * 10.0f;

    controller->UpdateKp(p);
    controller->UpdateKi(i);
    controller->UpdateKd(d);

    return true;
  }
};

}  // namespace controller