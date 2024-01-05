#pragma once

#include "packet.hpp"

namespace controller {

struct Joystick {
  int8_t x;
  int8_t y;

  Joystick() : x(0), y(0) {}

  void Parse(RawPacket const& packet) {
    x = packet[0] - 127;
    if (x == 1) x = 0;
    y = packet[1] - 127;
    if (y == 1) y = 0;
  }
  void Parse(RawPacketData const& packet) {
    x = packet[0] - 127;
    if (x == 1) x = 0;
    y = packet[1] - 127;
    if (y == 1) y = 0;
  }
};

}  // namespace controller
