#pragma once

#include "packet.hpp"

namespace controller {

struct Boolean {
  bool value_;

  Boolean() : value_(false) {}

  void Parse(RawPacket const& packet) { value_ = packet.element_id & 0x20; }
};

}  // namespace controller
