#pragma once

#include <ikarashiCAN_mk2.h>
#include "../../robotics/assembly/motor_with_encoder.hpp"

#include "../mdc.hpp"
#include "../../dcan.hpp"

class DrivingCANBus {
  ikarashiCAN_mk2 *ican_;
  int report_counter = 0;

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

  void ReportTo(DistributedCAN &can) {
    switch (report_counter) {
      case 0:
        mdc0_.ReportTo(can, 0);
        break;
      case 1:
        mdc1_.ReportTo(can, 1);
        break;
      case 2:
        mdc2_.ReportTo(can, 2);
        break;
    }
    report_counter = (report_counter + 1) % 3;
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
  robotics::assembly::MotorWithEncoder<float> &GetRevolver() {
    return mdc0_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetShotR() {
    return mdc1_.GetNode(0);
  }
  robotics::assembly::MotorWithEncoder<float> &GetShotL() {
    return mdc1_.GetNode(1);
  }
  robotics::assembly::MotorWithEncoder<float> &GetHorizontal() {
    return mdc1_.GetNode(2);
  }
  robotics::assembly::MotorWithEncoder<float> &GetElevation() {
    return mdc1_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetLoad() {
    return mdc2_.GetNode(0);
  }
};