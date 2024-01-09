#pragma once

#include "joystick.hpp"
#include "angle_joystick.hpp"
#include "boolean.hpp"
#include "pid.hpp"
#include "encoder.hpp"

#include "../robotics/component/swerve/swerve.hpp"

namespace controller::component {
struct SwerveController {
  Joystick move;
  AngleJoystick2D angle;
  Boolean rotation_pid_enabled;
  PID motor_0_pid;
  PID motor_1_pid;
  PID motor_2_pid;
  PID gyro_pid;

  bool Parse(RawPacket const& packet) {
    return move.Parse(packet) || angle.Parse(packet) ||
           rotation_pid_enabled.Parse(packet) || motor_0_pid.Parse(packet) ||
           motor_1_pid.Parse(packet) || motor_2_pid.Parse(packet) ||
           gyro_pid.Parse(packet);
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

struct Swerve {
  SwerveController controller;
  SwerveValueStore value_store;

  std::shared_ptr<robotics::component::swerve::Steering> steering;

  robotics::component::swerve::Steering const& GetSteering() {
    if (steering) {
      return *steering;
    }

    steering = std::make_shared<robotics::component::swerve::Steering>();
    controller.gyro_pid.ConnectGain(steering->angle_pid.GetController());
    controller.motor_0_pid.ConnectGain(
        steering->motors[0].GetAnglePIDController());
    controller.motor_1_pid.ConnectGain(
        steering->motors[1].GetAnglePIDController());
    controller.motor_2_pid.ConnectGain(
        steering->motors[2].GetAnglePIDController());
  }
};

}  // namespace controller::component