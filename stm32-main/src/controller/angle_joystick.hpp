#pragma once

#include "packet.hpp"

namespace controller {

struct AngleJoystick2D {
  uint8_t magnitude_;
  uint8_t angle_;  // clockwise

  AngleJoystick2D() : magnitude_(0), angle_(0) {}

  void Parse(RawPacket const& packet) {
    magnitude_ = packet[0];
    angle_ = packet[1];
  }
  void Parse(RawPacketData const& packet) {
    magnitude_ = packet[0];
    angle_ = packet[1];
  }
};

}