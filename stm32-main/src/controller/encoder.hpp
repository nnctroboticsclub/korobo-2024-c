#pragma once

#include "packet.hpp"

namespace controller {

struct Encoder {
  int assigned_id_;
  double value_;

  Encoder(int id) : value_(0), assigned_id_(id) {}

  bool Parse(RawPacket const& packet) {
    if (packet.element_id != (0x60 | assigned_id_)) {
      return false;
    }

    int16_t value = packet.data[0] << 8 | packet.data[1];

    value_ = value * 360.0f / 0x7FFF;
    return true;
  }
};

}  // namespace controller
