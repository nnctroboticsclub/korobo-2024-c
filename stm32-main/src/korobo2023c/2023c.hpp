#pragma once

#include "controller/joystick.hpp"
#include "controller/angle_joystick.hpp"
#include "controller/boolean.hpp"
#include "controller/pid.hpp"
#include "controller/encoder.hpp"
#include "controller/swerve.hpp"
#include "controller/float.hpp"
#include "controller/action.hpp"

#include "components/upper.hpp"

namespace korobo2023c {
struct Controller {
  struct Config {
    controller::swerve::SwerveController::Config swerve;
    Upper::Controller::Config upper;
    uint8_t soft_emc_id;

    uint8_t esc_factor_0_id;
    uint8_t esc_factor_1_id;
    uint8_t esc_factor_2_id;

    uint8_t steer_0_inverse_id;
    uint8_t steer_1_inverse_id;
    uint8_t steer_2_inverse_id;
  };

  controller::swerve::SwerveController swerve;
  Upper::Controller upper;
  controller::Boolean soft_emc;
  controller::Float esc_factor_0;
  controller::Float esc_factor_1;
  controller::Float esc_factor_2;

  controller::Action steer_0_inverse;
  controller::Action steer_1_inverse;
  controller::Action steer_2_inverse;

  Controller(Config const& config = {})
      : swerve(config.swerve),
        upper(config.upper),
        soft_emc(config.soft_emc_id),
        esc_factor_0(config.esc_factor_0_id),
        esc_factor_1(config.esc_factor_1_id),
        esc_factor_2(config.esc_factor_2_id),
        steer_0_inverse(config.steer_0_inverse_id),
        steer_1_inverse(config.steer_1_inverse_id),
        steer_2_inverse(config.steer_2_inverse_id)

  {}

  bool Pass(controller::RawPacket const& packet) {
    return swerve.Pass(packet) || upper.Pass(packet) || soft_emc.Pass(packet) ||
           esc_factor_0.Pass(packet) || esc_factor_1.Pass(packet) ||
           esc_factor_2.Pass(packet) || steer_0_inverse.Pass(packet) ||
           steer_1_inverse.Pass(packet) || steer_2_inverse.Pass(packet);
  }
};

template <typename T>
struct ValueStoreMain {
  struct Config {
    typename controller::swerve::SwerveValueStore<T>::Config swerve;
  };

  controller::swerve::SwerveValueStore<T> swerve;

  ValueStoreMain(Config const& config = {}) : swerve(config.swerve) {}

  bool Pass(controller::RawPacket const& packet) { return swerve.Pass(packet); }
};

}  // namespace korobo2023c