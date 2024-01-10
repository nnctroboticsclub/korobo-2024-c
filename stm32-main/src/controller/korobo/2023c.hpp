#pragma once

#include "../joystick.hpp"
#include "../angle_joystick.hpp"
#include "../boolean.hpp"
#include "../pid.hpp"
#include "../encoder.hpp"

#include "../swerve.hpp"

namespace controller {
struct Korobo2023Controller {
  struct Config {
    swerve::SwerveController::Config swerve;
  };

  swerve::SwerveController swerve;

  Korobo2023Controller(Config const& config = {}) : swerve(config.swerve) {}

  bool Parse(RawPacket const& packet) { return swerve.Parse(packet); }
};

struct Korobo2023MainValueStore {
  struct Config {
    swerve::SwerveValueStore::Config swerve;
  };

  swerve::SwerveValueStore swerve;

  Korobo2023MainValueStore(Config const& config = {}) : swerve(config.swerve) {}

  bool Parse(RawPacket const& packet) { return swerve.Parse(packet); }
};

}  // namespace controller