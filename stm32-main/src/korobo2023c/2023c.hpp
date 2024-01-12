#pragma once

#include "controller/joystick.hpp"
#include "controller/angle_joystick.hpp"
#include "controller/boolean.hpp"
#include "controller/pid.hpp"
#include "controller/encoder.hpp"
#include "controller/swerve.hpp"

namespace korobo::n2023c {
struct Controller {
  struct Config {
    controller::swerve::SwerveController::Config swerve;
  };

  controller::swerve::SwerveController swerve;

  Controller(Config const& config = {}) : swerve(config.swerve) {}

  bool Parse(controller::RawPacket const& packet) {
    return swerve.Parse(packet);
  }
};

template <typename T>
struct ValueStoreMain {
  struct Config {
    typename controller::swerve::SwerveValueStore<T>::Config swerve;
  };

  controller::swerve::SwerveValueStore<T> swerve;

  ValueStoreMain(Config const& config = {}) : swerve(config.swerve) {}

  bool Parse(controller::RawPacket const& packet) {
    return swerve.Parse(packet);
  }
};

}  // namespace korobo::n2023c