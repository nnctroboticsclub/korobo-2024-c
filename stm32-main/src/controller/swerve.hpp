#pragma once

#include "joystick.hpp"
#include "angle_joystick.hpp"
#include "boolean.hpp"
#include "pid.hpp"
#include "encoder.hpp"

#include "../robotics/component/swerve/swerve.hpp"

namespace controller::swerve {
struct SwerveController {
  JoyStick move;
  AngleJoystick2D angle;
  Boolean rotation_pid_enabled;
  PID motor_0_pid;
  PID motor_1_pid;
  PID motor_2_pid;
  PID gyro_pid;

  bool Parse(RawPacket const& packet) {
    return move.Pass(packet) || angle.Pass(packet) ||
           rotation_pid_enabled.Pass(packet) || motor_0_pid.Pass(packet) ||
           motor_1_pid.Pass(packet) || motor_2_pid.Pass(packet) ||
           gyro_pid.Pass(packet);
  }
};

struct SwerveValueStore {
  Encoder motor_0_encoder;
  Encoder motor_1_encoder;
  Encoder motor_2_encoder;

  bool Parse(RawPacket const& packet) {
    return motor_0_encoder.Pass(packet) || motor_1_encoder.Pass(packet) ||
           motor_2_encoder.Pass(packet);
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
    controller.gyro_pid.Connect(steering->angle_pid.GetController());
    controller.motor_0_pid.Connect(steering->motors[0].GetAnglePIDController());
    controller.motor_1_pid.Connect(steering->motors[1].GetAnglePIDController());
    controller.motor_2_pid.Connect(steering->motors[2].GetAnglePIDController());
    controller.move.Connect(steering->move.GetController());
  }
};

}  // namespace controller::swerve