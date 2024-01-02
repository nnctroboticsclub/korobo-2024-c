#pragma once

namespace controller {

using RawPacketData = std::vector<uint8_t>;

struct RawPacket {
  uint8_t element_id;
  RawPacketData data;

  RawPacket(RawPacketData const& raw_data) : element_id(raw_data[0]), data{} {
    data.reserve(raw_data.size() - 1);
    for (int i = 1; i < raw_data.size(); i++) {
      data.push_back(raw_data[i]);
    }
  }

  uint8_t operator[](int index) const { return data[index]; }
};

struct Joystick2D {
  int8_t x;
  int8_t y;

  Joystick2D() : x(0), y(0) {}

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

struct ControllerStatus {
  Joystick2D steer_move;
  AngleJoystick2D steer_angle;

  void Parse(RawPacket const& packet) {
    if (packet.element_id == 0x40) {
      steer_move.Parse(packet.data);
    } else if (packet.element_id == 0x41) {
      steer_angle.Parse(packet.data);
    }
  }
};
}  // namespace controller
