#pragma once

#include "packet.hpp"
#include "../robotics/filter/pid.hpp"

namespace controller {

class PID {
  using PIDController = robotics::filter::IPIDController;
  using PIDControllerPtr = std::shared_ptr<PIDController>;

  PIDControllerPtr controller;

 public:
  PID() : controller(nullptr) {}

  void ConnectGain(PIDControllerPtr controller) {
    this->controller = controller;
  }

  void Parse(RawPacket const& packet) {
    if (controller == nullptr) {
      return;
    }

    float p = packet.data[0] / 255.0f * 10.0f;
    float i = packet.data[1] / 255.0f * 10.0f;
    float d = packet.data[2] / 255.0f * 10.0f;

    controller->UpdateKp(p);
    controller->UpdateKi(i);
    controller->UpdateKd(d);
  }
};

}  // namespace controller