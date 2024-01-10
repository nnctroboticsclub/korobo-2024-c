#pragma once

#include "packet.hpp"
#include "controller_base.hpp"

namespace controller {

struct Encoder : public ControllerBase<double> {
  using ControllerBase::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == (0x60 | assigned_id_);
  }

  void Parse(RawPacket const& packet) override {
    int16_t value = packet.data[0] << 8 | packet.data[1];

    value = value * 360.0f / 0x7FFF;
  }
};

}  // namespace controller
