#pragma once

#include "packet.hpp"
#include "controller_base.hpp"

namespace controller {

template <typename T>
struct Encoder : public ControllerBase<T> {
  using ControllerBase<T>::ControllerBase;

  bool Filter(RawPacket const& packet) override {
    return packet.element_id == (0x60 | this->assigned_id_);
  }

  void Parse(RawPacket const& packet) override {
    if (packet.size() < 2) {
      printf("Invalid packet size for Encoder (id: %d)\n", assigned_id_);
      return;
    }

    int16_t value_ = packet.data[0] << 8 | packet.data[1];
    T value = value_ * 360.0f / 0x7FFF;

    this->SetValue(value);
  }
};

}  // namespace controller
