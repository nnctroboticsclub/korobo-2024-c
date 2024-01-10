#pragma once

#include "packet.hpp"
#include "controller_base.hpp"

#include "../robotics/sensor/encoder/base.hpp"

namespace controller {
template <typeneme T>
struct Encoder : public ControllerBase<T>,
                 public robotics::sensor::encoder::Absolute<T> {
  using ControllerBase::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == (0x60 | assigned_id_);
  }

  void Parse(RawPacket const& packet) override {
    int16_t value = packet.data[0] << 8 | packet.data[1];

    value = value * 360.0f / 0x7FFF;
  }

  T GetAngle() override { return value; }

  std::shared_ptr<robotics::sensor::encoder::Absolute<T>> GetEncoder() {
    return std::make_shared<Encoder>(*this);
  }
};

}  // namespace controller
