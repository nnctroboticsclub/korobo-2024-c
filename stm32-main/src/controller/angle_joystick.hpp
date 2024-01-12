#pragma once

#include "controller_base.hpp"
#include "../robotics/types/angle_joystick_2d.hpp"

namespace controller {

struct AngleJoystick2D : public ControllerBase<robotics::AngleStick2D> {
  using ControllerBase::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == (0x40 | assigned_id_);
  }

  void Parse(RawPacket const& packet) override {
    robotics::AngleStick2D value;

    value.magnitude = packet[0];
    value.angle = packet[1];

    this->SetValue(value);
  }
};

}  // namespace controller