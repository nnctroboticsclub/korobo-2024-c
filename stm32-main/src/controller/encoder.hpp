#pragma once

#include "packet.hpp"

namespace controller {

struct Encoder {
  double value_;

  Encoder() : value_(0) {}

  void Parse(RawPacket const& packet) {
    int16_t value = packet.data[0] << 8 | packet.data[1];

    value_ = value * 360.0f / 0x7FFF;
  }
};

}  // namespace controller
