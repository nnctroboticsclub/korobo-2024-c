#pragma once

#include <math.h>
#include <memory>

#include "../../types/vector.hpp"
#include "../../fusion/angled_motor.hpp"
#include "../../output/motor.hpp"

namespace robotics::component::swerve {

class Motor {
 public:
  struct Config {
    std::unique_ptr<output::Motor<float>> drive;
    std::unique_ptr<fusion::AngledMotor<float>> steer;
    int angle_deg;
  };

 private:
  std::unique_ptr<output::Motor<float>> drive_;
  std::unique_ptr<fusion::AngledMotor<float>> steer_;

  Vector<float, 2> normal_vector_;

 public:
  Motor(Config&& config) {
    drive_ = std::move(config.drive);
    steer_ = std::move(config.steer);
    normal_vector_[0] = std::cos(config.angle_deg * M_PI / 180);
    normal_vector_[1] = std::sin(config.angle_deg * M_PI / 180);
  }

  input::IInputController<PIDGains>* GetAnglePIDController() {
    return steer_->GetPIDController();
  }

  void SetPowerRaw(Vector<float, 2> vector) {
    float magnitude = vector.Magnitude();
    float angle = std::atan2(vector[1], vector[0]) * 180 / M_PI;

    steer_->SetTargetAngle(angle);
    drive_->SetSpeed(magnitude);
  }

  /**
   * @brief Set Driving Power of the Swerve Drive
   *
   * @param velocity Velocity Vector
   * @param rotation Rotation Power (0.0 ~ 360.0)
   */
  void SetDrivePower(Vector<float, 2> velocity, float rotation) {
    auto vector = velocity + normal_vector_ * rotation / 90;
    SetPowerRaw(vector);
  }

  void Update(float dt) { steer_->Update(dt); }
};

}  // namespace robotics::component::swerve