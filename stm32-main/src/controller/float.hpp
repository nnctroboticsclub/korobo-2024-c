#pragma once

#include "packet.hpp"
#include "controller_base.hpp"

namespace controller {
struct Float : public ControllerBase<float> {
  using ControllerBase::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == assigned_id_;
  }

  void Parse(RawPacket const& packet) override {
    float value;
    value = (packet[0] << 8 | packet[1]) / 65536.0f;

    this->SetValue(value);
  }
};

}  // namespace controller
