#pragma once

#include "packet.hpp"
#include "../robotics/input/input.hpp"
#include "../robotics/types/joystick_2d.hpp"
#include "controller_base.hpp"

namespace controller {
struct JoyStick : public ControllerBase<robotics::JoyStick2D> {
  using ControllerBase::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == (0x40 | assigned_id_);
  }

  void Parse(RawPacket const& packet) override {
    value[0] = (packet[0] - 127) / 127.0f;
    value[1] = (packet[1] - 127) / 127.0f;
  }
};

}  // namespace controller
