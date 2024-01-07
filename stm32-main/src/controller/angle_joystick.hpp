#pragma once

#include "packet.hpp"

namespace controller {

struct AngleJoystick2D {
  int assigned_id_;
  uint8_t magnitude_;
  uint8_t angle_;  // clockwise

  AngleJoystick2D(int id) : assigned_id_(id), magnitude_(0), angle_(0) {}

  bool Parse(RawPacket const& packet) {
    if (packet.element_id != (0x40 | assigned_id_)) {
      return false;
    }
    magnitude_ = packet[0];
    angle_ = packet[1];
    return false;
  }
};

}  // namespace controller