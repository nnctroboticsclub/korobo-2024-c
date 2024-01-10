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
  std::array<Motor, 3> motors;
  std::shared_ptr<sensor::gyro::Base> gyro;
  filter::PID<float> angle_pid{0.7f, 0.30f, 0.15f};

  input::Input<bool> rotation_pid_enabled;
  input::Input<types::JoyStick2D> move;
  input::Input<types::AngleStick2D> angle;

 private:
  filter::AngleNormalizer<float> rot_in_normalizer;
  filter::AngleNormalizer<float> self_rot_y_normalizer;

 private:
  void Update(float dt) {
    auto velocity = move.GetValue();
    float rotation_in_raw = angle.GetValue().angle;
    float angle_power = angle.GetValue().magnitude;

    // Rotation PID
    float rotation_in = rot_in_normalizer.Update(rotation_in_raw, angle_power);
    float rotation_fb =
        self_rot_y_normalizer.Update(gyro->GetHorizontalOrientation());
    float rotation = rotation_pid_enabled.GetValue()
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