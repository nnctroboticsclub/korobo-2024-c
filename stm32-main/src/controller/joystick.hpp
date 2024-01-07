#pragma once

#include "packet.hpp"

namespace controller {

struct Joystick {
  int assigned_id_;
  int8_t x;
  int8_t y;

  Joystick(int id) : assigned_id_(id), x(0), y(0) {}

  bool Parse(RawPacket const& packet) {
    if (packet.element_id != (0x40 | assigned_id_)) {
      return false;
    }
    x = packet[0] - 127;
    if (x == 1) x = 0;
    y = packet[1] - 127;
    if (y == 1) y = 0;

    return true;
  }
};

}  // namespace controller
