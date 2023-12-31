#pragma once

#include "Adafruit_BNO055.h"
#include "rtos.h"

class Gyro {
  Adafruit_BNO055 bno055_;
  rtos::Thread thread;

  void ThreadMain() {
    if (!bno055_.begin(Adafruit_BNO055::OPERATION_MODE_ACCGYRO)) {
      printf(
          "\e[41m   \e[m\e[31m BNO055 Initialization failed \e[41m   \e[m\n");
    }

    imu::Vector<3> velocity;
    Timer timer{};  // Used to calculate delta time (dt)
    int prev = 0;

    timer.start();
    prev = timer.read_ms();

    while (true) {
      imu::Vector<3> accel =
          bno055_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      auto quat = bno055_.getQuat();

      auto accel_world = quat.rotateVector(accel);

      int now = timer.read_ms();
      float dt = (now - prev) / 1000.0f;  // seconds
      prev = now;

      // velocity += accel_world * dt;

      printf("accel_world: %f %f %f\n", accel_world.x(), accel_world.y(),
             accel_world.z());
      // printf("velocity: %f %f %f\n", velocity.x(), velocity.y(),
      // velocity.z());

      rtos::ThisThread::sleep_for(100ms);
    }
  }

 public:
  Gyro(I2C &i2c) : bno055_(-1, BNO055_ADDRESS_A, &i2c), thread() {}

  void Init() { thread.start(callback(this, &Gyro::ThreadMain)); }
};