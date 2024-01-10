#pragma once

#include "../joystick.hpp"
#include "../angle_joystick.hpp"
#include "../boolean.hpp"
#include "../pid.hpp"
#include "../encoder.hpp"

#include "../swerve.hpp"

namespace controller {
struct Korobo2023Controller {
  swerve::SwerveController swerve;

  bool Parse(RawPacket const& packet) { return swerve.Parse(packet); }
};

struct Korobo2023MainValueStore {
  swerve::SwerveValueStore swerve;

  bool Parse(RawPacket const& packet) { return swerve.Parse(packet); }
};

}  // namespace controller