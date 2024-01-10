#pragma once

#include "../filter/pid.hpp"
#include "../output/motor.hpp"
#include "../sensor/encoder/base.hpp"

namespace robotics::fusion {
template <typename T>
class AngledMotorConfig {
  std::shared_ptr<sensor::encoder::Absolute<T>> encoder;
  std::unique_ptr<output::Motor<T>> motor;
};

template <typename T>
class AngledMotorConfigBuilder {
  AngledMotorConfig<T> config;

 public:
  AngledMotorConfigBuilder& Encoder(
      std::shared_ptr<sensor::encoder::Absolute<T>> encoder) {
    config.encoder = std::move(encoder);
    return *this;
  }

  AngledMotorConfigBuilder& Motor(std::unique_ptr<output::Motor<T>> motor) {
    config.motor = std::move(motor);
    return *this;
  }

  AngledMotorConfig<T> Build() { return std::move(config); }
};

template <typename T>
class AngledMotor {
 public:
  filter::PID<float> angle_;

 private:
  float target_angle_;
  std::shared_ptr<sensor::encoder::Absolute<T>> encoder_;
  std::unique_ptr<output::Motor<T>> motor_;

 public:
  AngledMotor(AngledMotorConfig<T>&& config) {
    encoder_ = std::move(config.encoder);
    motor_ = std::move(config.motor);
  }

  input::IInputController<PIDGains>* GetPIDController() {
    return angle_.GetController();
  }

  void SetTargetAngle(float angle) { target_angle_ = angle; }

  void Update(float dt) {
    float feedback = encoder_->GetAngle();
    float output = angle_.Update(target_angle_, feedback, dt);
    motor_->SetSpeed(output);
  }
};
}  // namespace robotics::fusion