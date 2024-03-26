#pragma once

#include <ikarashiCAN_mk2.h>
#include <robotics/assembly/motor_pair.hpp>
#include <dcan.hpp>

#include "../mdc.hpp"

class DrivingCANBus {
  ikarashiCAN_mk2 *ican_;
  int report_counter = 0;

 public:
  MDC mdc0_;
  MDC mdc1_;
  MDC mdc2_;

  DrivingCANBus(ikarashiCAN_mk2 *ican);

  void Init();
  void Tick();
  void Send();

  void ReportTo(DistributedCAN &can);

  robotics::assembly::MotorPair<float> &GetSwerveRot0();
  robotics::assembly::MotorPair<float> &GetSwerveRot1();
  robotics::assembly::MotorPair<float> &GetSwerveRot2();
  robotics::assembly::MotorPair<float> &GetRevolver();

  robotics::assembly::MotorPair<float> &GetShotR();
  robotics::assembly::MotorPair<float> &GetShotL();
  robotics::assembly::MotorPair<float> &GetHorizontal();
  robotics::assembly::MotorPair<float> &GetElevation();

  robotics::assembly::MotorPair<float> &GetLoad();
};