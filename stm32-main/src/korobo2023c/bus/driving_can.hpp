#pragma once

#include <ikarashiCAN_mk2.h>
#include "../../robotics/assembly/motor_with_encoder.hpp"

#include "../mdc.hpp"

class DrivingCANBus {
  ikarashiCAN_mk2 *ican_;

 public:
  MDC mdc0_;
  MDC mdc1_;
  MDC mdc2_;

  DrivingCANBus(ikarashiCAN_mk2 *ican)
      : ican_(ican), mdc0_(ican, 1), mdc1_(ican, 2), mdc2_(ican, 3) {}

  void Init() { ican_->read_start(); }

  void Tick() {
    mdc0_.Tick();
    mdc1_.Tick();
    mdc2_.Tick();
  }

  void Send() {
    mdc0_.Send();
    mdc1_.Send();
    mdc2_.Send();
  }

  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot0() {
    return mdc0_.GetNode(0);
  }
  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot1() {
    return mdc0_.GetNode(1);
  }
  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot2() {
    return mdc0_.GetNode(2);
  }
  robotics::assembly::MotorWithEncoder<float> &GetShotL() {
    return mdc0_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetRevolver() {
    return mdc1_.GetNode(0);
  }
  robotics::assembly::MotorWithEncoder<float> &GetLoad() {
    return mdc1_.GetNode(1);
  }
  robotics::assembly::MotorWithEncoder<float> &GetHorizontal() {
    return mdc1_.GetNode(2);
  }
  robotics::assembly::MotorWithEncoder<float> &GetElevation() {
    return mdc1_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetShotR() {
    return mdc2_.GetNode(0);
  }
};