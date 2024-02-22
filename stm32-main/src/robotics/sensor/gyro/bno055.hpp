#pragma once

#include <bno055.h>
#include <rtos.h>

#include "base.hpp"
#include "../../types/vector.hpp"

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace robotics::sensor::gyro {
class BNO055 : public Base {
  ::BNO055 bno055_;
  rtos::Thread *thread;

  Timer timer;

  void ThreadMain() {
    bno055_.reset();

    bno055_.setpowermode(POWER_MODE_NORMAL);
    bno055_.setmode(OPERATION_MODE_NDOF);

    bno055_.set_angle_units(DEGREES);
    bno055_.set_accel_units(MPERSPERS);

    timer.start();
    while (true) {
      bno055_.get_angles();

      this->SetValue(bno055_.euler.yaw);
    }
  }

 public:
  BNO055(PinName SDA, PinName SDL) : bno055_(SDA, SDL) {}

  bool Init() {
    if (!bno055_.check()) {
      return false;
    }

    thread = new rtos::Thread(osPriorityNormal, 1024 * 4);
    thread->start(callback(this, &BNO055::ThreadMain));
    return true;
  }
};
}  // namespace robotics::sensor::gyro