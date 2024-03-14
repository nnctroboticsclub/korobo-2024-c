#pragma once

#include "controller/joystick.hpp"
#include "controller/angle_joystick.hpp"
#include "controller/boolean.hpp"
#include "controller/pid.hpp"
#include "controller/encoder.hpp"
#include "controller/swerve.hpp"
#include "controller/float.hpp"
#include "controller/action.hpp"

namespace korobo::n2023c {
struct Controller {
  struct Config {
    controller::swerve::SwerveController::Config swerve;
    uint8_t shot_joystick_id;
    uint8_t do_shot_id;
    uint8_t shot_speed_id;
    uint8_t max_elevation_id;

    uint8_t esc_factor_0_id;
    uint8_t esc_factor_1_id;
    uint8_t esc_factor_2_id;

    uint8_t revolver_change_id;

    uint8_t steer_0_inverse_id;
    uint8_t steer_1_inverse_id;
    uint8_t steer_2_inverse_id;
  };

  controller::swerve::SwerveController swerve;
  controller::JoyStick shot;
  controller::Boolean do_shot;
  controller::Float shot_speed;
  controller::Float max_elevation;
  controller::Float esc_factor_0;
  controller::Float esc_factor_1;
  controller::Float esc_factor_2;
  controller::Action revolver_change;

  controller::Action steer_0_inverse;
  controller::Action steer_1_inverse;
  controller::Action steer_2_inverse;

  Controller(Config const& config = {})
      : swerve(config.swerve),
        shot(config.shot_joystick_id),
        do_shot(config.do_shot_id),
        shot_speed(config.shot_speed_id),
        max_elevation(config.max_elevation_id),
        esc_factor_0(config.esc_factor_0_id),
        esc_factor_1(config.esc_factor_1_id),
        esc_factor_2(config.esc_factor_2_id),
        revolver_change(config.revolver_change_id),
        steer_0_inverse(config.steer_0_inverse_id),
        steer_1_inverse(config.steer_1_inverse_id),
        steer_2_inverse(config.steer_2_inverse_id) {}

  bool Pass(controller::RawPacket const& packet) {
    return swerve.Pass(packet) || shot.Pass(packet) || do_shot.Pass(packet) ||
           shot_speed.Pass(packet) || max_elevation.Pass(packet) ||
           esc_factor_0.Pass(packet) || esc_factor_1.Pass(packet) ||
           esc_factor_2.Pass(packet);
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

}  // namespace korobo::n2023c