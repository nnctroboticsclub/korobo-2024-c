#pragma once

#include "joystick.hpp"
#include "angle_joystick.hpp"
#include "boolean.hpp"
#include "pid.hpp"
#include "encoder.hpp"

#include "../robotics/component/swerve/swerve.hpp"

namespace controller::swerve {
struct SwerveController {
  struct Config {
    int joystick_id = 0;
    int angle_joystick_id = 1;
    int rotation_pid_enabled_id = 1;
    int motor_0_pid_id = 0;
    int motor_1_pid_id = 1;
    int motor_2_pid_id = 2;
    int angle_pid_id = 3;
  };

  JoyStick move;
  AngleJoystick2D angle;
  Boolean rotation_pid_enabled;
  PID motor_0_pid;
  PID motor_1_pid;
  PID motor_2_pid;
  PID angle_pid;

  SwerveController(Config const& config = {})
      : move(config.joystick_id),
        angle(config.angle_joystick_id),
        rotation_pid_enabled(config.rotation_pid_enabled_id),
        motor_0_pid(config.motor_0_pid_id),
        motor_1_pid(config.motor_1_pid_id),
        motor_2_pid(config.motor_2_pid_id),
        angle_pid(config.angle_pid_id) {}

  bool Parse(RawPacket const& packet) {
    return move.Pass(packet) || angle.Pass(packet) ||
           rotation_pid_enabled.Pass(packet) || motor_0_pid.Pass(packet) ||
           motor_1_pid.Pass(packet) || motor_2_pid.Pass(packet) ||
           angle_pid.Pass(packet);
  }

  void Connect(robotics::component::Swerve& swerve) {
    move.Connect(swerve.move.GetController());
    angle.Connect(swerve.angle.GetController());
    rotation_pid_enabled.Connect(
        swerve.rotation_direct_mode_enabled.GetController());
    motor_0_pid.Connect(swerve.motors[0].GetAnglePIDController());
    motor_1_pid.Connect(swerve.motors[1].GetAnglePIDController());
    motor_2_pid.Connect(swerve.motors[2].GetAnglePIDController());
    angle_pid.Connect(swerve.angle_pid.GetController());
  }
};

template <typename T>
struct SwerveValueStore {
  struct Config {
    int motor_0_encoder_id = 0;
    int motor_1_encoder_id = 1;
    int motor_2_encoder_id = 2;
  };

  Encoder<T> motor_0_encoder;
  Encoder<T> motor_1_encoder;
  Encoder<T> motor_2_encoder;

  SwerveValueStore(Config const& config = {})
      : motor_0_encoder(config.motor_0_encoder_id),
        motor_1_encoder(config.motor_1_encoder_id),
        motor_2_encoder(config.motor_2_encoder_id) {}

  bool Parse(RawPacket const& packet) {
    return motor_0_encoder.Pass(packet) || motor_1_encoder.Pass(packet) ||
           motor_2_encoder.Pass(packet);
  }
};

}  // namespace controller::swerve