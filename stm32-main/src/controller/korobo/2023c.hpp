#pragma once

#include "../joystick.hpp"
#include "../angle_joystick.hpp"
#include "../boolean.hpp"
#include "../pid.hpp"

namespace controller {
struct Korobo2023Controller {
  Joystick steer_move;
  AngleJoystick2D steer_angle;
  Boolean steer_rotation_pid_enabled;
  PID steer_motor_0_pid;
  PID steer_motor_1_pid;
  PID steer_motor_2_pid;
  PID steer_gyro_pid;

  void Parse(RawPacket const& packet) {
    if (packet.element_id == 0x40) {
      steer_move.Parse(packet.data);
    } else if (packet.element_id == 0x41) {
      steer_angle.Parse(packet.data);
    } else if ((packet.element_id & 0xdf) == 0x01) {
      steer_rotation_pid_enabled.Parse(packet);
    } else if (packet.element_id == 0xA0) {
      steer_motor_0_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA1) {
      steer_motor_1_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA2) {
      steer_motor_2_pid.Parse(packet.data);
    } else if (packet.element_id == 0xA3) {
      steer_gyro_pid.Parse(packet.data);
    } else {
      printf("unknown packet: %02x\n", packet.element_id);
    }
  }
};
}  // namespace controller