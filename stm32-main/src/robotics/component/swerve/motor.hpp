#pragma once

#include <math.h>
#include <memory>

#include "../../types/vector.hpp"
#include "../../fusion/angled_motor.hpp"
#include "../../output/motor.hpp"

namespace robotics::component::swerve {

class Motor {
  std::shared_ptr<output::Motor<float>> drive_;
  fusion::AngledMotor<float> steer_;

  Vector<float, 2> normal_vector_;

 public:
  Motor(std::shared_ptr<output::Motor<float>> drive,
        fusion::AngledMotor<float> steer, float angle_deg)
      : drive_(drive), steer_(steer) {
    normal_vector_ = Vector<float, 2>(std::cos(angle_deg * M_PI / 180),
                                      std::sin(angle_deg * M_PI / 180));
  }

  input::IInputController<PIDGains> *GetAnglePIDController() {
    return steer_.GetPIDController();
  }

  void SetPowerRaw(Vector<float, 2> vector) {
    float magnitude = vector.Magnitude();
    float angle = std::atan2(vector[1], vector[0]) * 180 / M_PI;

    steer_.SetTargetAngle(angle);
    drive_->SetSpeed(magnitude);
  }

  /**
   * @brief Set Driving Power of the Swerve Drive
   *
   * @param velocity Velocity Vector
   * @param rotation Rotation Power (0.0 ~ 360.0)
   */
  void SetDrivePower(Vector<int8_t, 2> velocity, float rotation) {
    auto vector = velocity + normal_vector_ * rotation / 90;
    SetPowerRaw(vector);
  }

  void Update(float dt) { steer_.Update(dt); }
};

}  // namespace robotics::component::swerve