#pragma once

#include "controller/joystick.hpp"
#include "controller/angle_joystick.hpp"
#include "controller/boolean.hpp"
#include "controller/pid.hpp"
#include "controller/encoder.hpp"
#include "controller/swerve.hpp"
#include "controller/float.hpp"

namespace korobo::n2023c {
struct Controller {
  struct Config {
    controller::swerve::SwerveController::Config swerve;
    int shot_joystick_id;
    int do_shot_id;
    int shot_speed_id;
    int max_elevation_id;
  };

  controller::swerve::SwerveController swerve;
  controller::JoyStick shot;
  controller::Boolean do_shot;
  controller::Float shot_speed;
  controller::Float max_elevation;

  Controller(Config const& config = {})
      : swerve(config.swerve),
        shot(config.shot_joystick_id),
        do_shot(config.do_shot_id),
        shot_speed(config.shot_speed_id),
        max_elevation(config.max_elevation_id) {}

  bool Pass(controller::RawPacket const& packet) {
    return swerve.Pass(packet) || shot.Pass(packet) || do_shot.Pass(packet) ||
           shot_speed.Pass(packet) || max_elevation.Pass(packet);
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