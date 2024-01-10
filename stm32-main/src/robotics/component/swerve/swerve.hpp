#pragma once

#include <memory>

#include "motor.hpp"

#include "../../filter/pid.hpp"
#include "../../filter/angle_smoother.hpp"
#include "../../fusion/angled_motor.hpp"
#include "../../sensor/gyro/base.hpp"
#include "../../input/input.hpp"
#include "../../types/angle_joystick_2d.hpp"
#include "../../types/joystick_2d.hpp"
#include "../../types/vector.hpp"

namespace robotics::component {
namespace swerve {
class Swerve {
 public:
  struct Config {
    std::array<Motor::Config, 3> motors;
    std::shared_ptr<sensor::gyro::Base> gyro;
  };

  filter::PID<float> angle_pid{0.7f, 0.30f, 0.15f};

  input::Input<bool> rotation_direct_mode_enabled;
  input::Input<types::JoyStick2D> move;
  input::Input<types::AngleStick2D> angle;

 private:
  std::array<Motor, 3> motors;
  std::shared_ptr<sensor::gyro::Base> gyro;

  filter::AngleNormalizer<float> rot_in_normalizer;
  filter::AngleNormalizer<float> self_rot_y_normalizer;

 public:
  Swerve(Config&& config)
      : motors({Motor(std::move(config.motors[0])),
                Motor(std::move(config.motors[1])),
                Motor(std::move(config.motors[2]))}),
        gyro(config.gyro) {}

  void Update(float dt) {
    auto velocity = move.GetValue();
    float rotation_in_raw = angle.GetValue().angle;
    float angle_power = angle.GetValue().magnitude;

    // Rotation PID
    float rotation_in = rot_in_normalizer.Update(rotation_in_raw, angle_power);
    float rotation_fb =
        self_rot_y_normalizer.Update(gyro->GetHorizontalOrientation());
    float rotation = rotation_direct_mode_enabled.GetValue()
                         ? rotation_in
                         : angle_pid.Update(rotation_in, rotation_fb, dt);

    for (auto& motor : motors) {
      motor.SetDrivePower(velocity, rotation);
    }
  }
};
}  // namespace swerve

using Swerve = swerve::Swerve;
}  // namespace robotics::component