#pragma once

#include "packet.hpp"

namespace controller {

struct Boolean {
  int assigned_id_;
  bool value_;

  Boolean(int id) : assigned_id_(id), value_(false) {}

  bool Parse(RawPacket const& packet) {
    if ((packet.element_id & 0xDF) != assigned_id_) {
      return false;
    }
    value_ = packet.element_id & 0x20;
    return true;
  }
};

}  // namespace controller
