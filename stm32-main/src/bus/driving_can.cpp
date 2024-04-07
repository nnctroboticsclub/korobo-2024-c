#include "driving_can.hpp"

DrivingCANBus::DrivingCANBus(ikarashiCAN_mk2 *ican)
    : ican_(ican), mdc0_(ican, 1), mdc1_(ican, 2), mdc2_(ican, 3) {}

void DrivingCANBus::Init() { ican_->read_start(); }

void DrivingCANBus::Tick() {
  mdc0_.Tick();
  mdc1_.Tick();
  mdc2_.Tick();
}

void DrivingCANBus::Send() {
  mdc0_.Send();
  mdc1_.Send();
  mdc2_.Send();
}

void DrivingCANBus::ReportTo(robotics::network::DistributedCAN &can) {
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

robotics::assembly::MotorPair<float> &DrivingCANBus::GetSwerveRot0() {
  return mdc0_.GetNode(0);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetSwerveRot1() {
  return mdc0_.GetNode(1);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetSwerveRot2() {
  return mdc0_.GetNode(2);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetRevolver() {
  return mdc0_.GetNode(3);
}

robotics::assembly::MotorPair<float> &DrivingCANBus::GetShotR() {
  return mdc1_.GetNode(0);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetShotL() {
  return mdc1_.GetNode(1);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetHorizontal() {
  return mdc1_.GetNode(2);
}
robotics::assembly::MotorPair<float> &DrivingCANBus::GetElevation() {
  return mdc1_.GetNode(3);
}

robotics::assembly::MotorPair<float> &DrivingCANBus::GetLoad() {
  return mdc2_.GetNode(0);
}
