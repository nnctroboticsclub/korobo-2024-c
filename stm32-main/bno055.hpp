#pragma once

#include "bno055.h"
#include "rtos.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

class Gyro {
  BNO055 bno055_;
  rtos::Thread thread;
  rtos::Thread monitor_thread;

  Timer timer;

  float orientation_ = -1;
  float velocity_x_ = 0;
  float velocity_y_ = 0;

  void ThreadMain() {
    if (!bno055_.check()) {
      printf("BNO055 not detected\n");
      return;
    }

    bno055_.reset();

    bno055_.setpowermode(POWER_MODE_NORMAL);
    bno055_.setmode(OPERATION_MODE_IMUPLUS);

    bno055_.set_angle_units(DEGREES);
    bno055_.set_accel_units(MPERSPERS);

    timer.start();
    while (true) {
      bno055_.get_angles();
      bno055_.get_accel();

      float yaw = bno055_.euler.yaw;
      float pitch = bno055_.euler.pitch;
      float roll = bno055_.euler.roll;

      float ax = bno055_.accel.x;
      float ay = bno055_.accel.y;
      float az = bno055_.accel.z;

      // Calculate world acceleration
      float cy = cos(yaw * PI / 180);
      float sy = sin(yaw * PI / 180);
      float cp = cos(pitch * PI / 180);
      float sp = sin(pitch * PI / 180);
      float cr = cos(roll * PI / 180);
      float sr = sin(roll * PI / 180);

      // sympy でゴリゴリ計算した
      float fax = -cp * sy * ay + ax * (cr * cy - sp * sr * sy) -
                  az * (cr * sp * sy + cy * sr);
      float fay = cp * cy * ay + ax * (cr * sy + cy * sp * sr) +
                  az * (cr * cy * sp - sr * sy);

      if (-0.05 < fax && fax < 0.05) {
        fax = 0;
      }

      if (-0.05 < fay && fay < 0.05) {
        fay = 0;
      }

      int dt = timer.read_ms();
      timer.reset();

      orientation_ = yaw;

      /**
       * a [m/s^2], dt [ms] -> v [m/s]
       * dt [ms] -> dt * 1E-3 [s]
       * a [m/s^2] * (dt * 1E-3) [s] -> v [m/s]
       */
      velocity_x_ += fax * dt * 0.001;
      velocity_y_ += fay * dt * 0.001;
    }
  }

  void Monitor() {
    while (true) {
      printf("Yaw: %+6.2f Velocity: (%+5.2f, %+5.2f)\n", orientation_,
             velocity_x_, velocity_y_);
      rtos::ThisThread::sleep_for(50ms);
    }
  }

 public:
  Gyro(PinName SDA, PinName SDL) : bno055_(SDA, SDL) {}

  void Init(bool monitor = false) {
    thread.start(callback(this, &Gyro::ThreadMain));
    if (monitor) {
      monitor_thread.start(callback(this, &Gyro::Monitor));
    }
  }
};