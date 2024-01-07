#pragma once

#include "../joystick.hpp"
#include "../angle_joystick.hpp"
#include "../boolean.hpp"
#include "../pid.hpp"
#include "../encoder.hpp"

namespace controller {
struct Swerve {
  Joystick move;
  AngleJoystick2D angle;
  Boolean rotation_pid_enabled;
  PID motor_0_pid;
  PID motor_1_pid;
  PID motor_2_pid;
  PID gyro_pid;

  bool Parse(RawPacket const& packet) {
    if (packet.element_id == 0x40) {
      move.Parse(packet.data);
    } else if (packet.element_id == 0x41) {
      angle.Parse(packet.data);
    } else if ((packet.element_id & 0xdf) == 0x01) {
      rotation_pid_enabled.Parse(packet);
    } else if (packet.element_id == 0xA0) {
      motor_0_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA1) {
      motor_1_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA2) {
      motor_2_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA3) {
      gyro_pid.Parse(packet.data);
    } else {
      return false;
    }
    return true;
  }
};

struct SwerveValueStore {
  Encoder motor_0_encoder;
  Encoder motor_1_encoder;
  Encoder motor_2_encoder;

  bool Parse(RawPacket const& packet) {
    return motor_0_encoder.Parse(packet) || motor_1_encoder.Parse(packet) ||
           motor_2_encoder.Parse(packet);
  }
};

}  // namespace controller