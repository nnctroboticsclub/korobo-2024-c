#include "communication.hpp"

#include <cmath>
#include <mbed-robotics/simple_can.hpp>

void Communication::InitCAN() {
  printf("\e[1;32m|\e[m \e[32m-\e[m Initializing CAN (Com)\n");
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Initializing CAN Driver\n");
  can_.Init();
  driving_->Init();
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Adding Handlers\n");
  can_.OnMessage(0x40, [this](std::vector<uint8_t> data) {  //
    if (data.size() < 1) {
      printf("C< Invaid (0 bytes)\n");
      return;
    }
    std::stringstream ss;
    for (auto byte : data) {
      ss << std::setw(2) << std::hex << (int)byte << " ";
    }
    // printf("C< %s(%d bytes)\n", ss.str().c_str(), data.size());
    controller_status_.Pass(data);
  });
  can_.OnMessage(0x61, [this](std::vector<uint8_t> data) {  //
    value_store_.Pass(data);
  });
}

void Communication::InitGyro() {
  SetStatus(robotics::network::DistributedCAN::Statuses::kInitializingGyro);
  printf("\e[1;32m|\e[m \e[32m-\e[m Initializing Gyro\n");
  auto gyro_init_status = gyro_.Init();
  if (!gyro_init_status) {
    printf(
        "\e[1;32m|\e[m \e[32m-\e[m \e[1;31m=> !!!\e[m BNO055 Detection "
        "failed.\e[m\n");
  }
}

void Communication::InitBLDC() {
  printf("\e[1;32m|\e[m \e[32m-\e[m Initializing ESC\n");
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Max Pulsewidth\n");
  SetStatus(robotics::network::DistributedCAN::Statuses::kInitializingESC0);
  bldc[0].Init0();
  bldc[1].Init0();
  bldc[2].Init0();
  ThisThread::sleep_for(2s);
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Min Pulsewidth\n");
  SetStatus(robotics::network::DistributedCAN::Statuses::kInitializingESC1);
  bldc[0].Init1();
  bldc[1].Init1();
  bldc[2].Init1();
  ThisThread::sleep_for(1s);
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m OK\n");
}

void Communication::ReportBLDC() {
  std::vector<uint8_t> report(4);
  report.reserve(4);

  report[0] = 0x30 | 0x08;
  report[1] = std::max(std::min(bldc[0].GetSpeed(), 1.0f), -1.0f) * 127 + 128;
  report[2] = std::max(std::min(bldc[1].GetSpeed(), 1.0f), -1.0f) * 127 + 128;
  report[3] = std::max(std::min(bldc[2].GetSpeed(), 1.0f), -1.0f) * 127 + 128;

  auto ret = can_.Send(0xa0, report);
  if (ret != 1) {
    printf("COM Report: Sending the BLDC report is failed. (opcode: 0x38)\n");
  }
}

Communication::Communication(Config &config)
    : can_(config.can.id,
           std::make_shared<robotics::network::SimpleCAN>(
               config.can.rx, config.can.tx, config.can.freqency)),
      driving_(std::make_unique<DrivingCANBus>(new ikarashiCAN_mk2(
          config.driving_can.rx, config.driving_can.tx, 0))),
      controller_status_(config.controller_ids),
      value_store_(config.value_store_ids),
      gyro_(config.i2c.sda, config.i2c.scl),
      bldc{
          {std::make_shared<robotics::driver::PWM>(
               config.swerve_esc_pins.swerve_pin_m0),
           1000, 2000},
          {std::make_shared<robotics::driver::PWM>(
               config.swerve_esc_pins.swerve_pin_m1),
           1000, 2000},
          {std::make_shared<robotics::driver::PWM>(
               config.swerve_esc_pins.swerve_pin_m2),
           1000, 2000},
      } {}

void Communication::SendNonReactiveValues() {
  this->driving_->Tick();
  this->driving_->Send();
}

void Communication::Init() {
  InitCAN();
  InitBLDC();
  InitGyro();

  printf("\e[1;32m|\e[m \e[32m-\e[m Setting Initial value\n");
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Set Dummy value\n");
  controller_status_.swerve.angle_out.SetValue({0.1, 0.1});
  controller_status_.swerve.move.SetValue({0.1, 0.1});
  ThisThread::sleep_for(10ms);
  printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Reset Initial value\n");
  controller_status_.swerve.angle_out.SetValue({0, 0});
  controller_status_.swerve.move.SetValue({0, 0});
}

void Communication::SetStatus(
    robotics::network::DistributedCAN::Statuses status) {
  can_.SetStatus(status);
}

void Communication::LinkToSwerve(SwerveComponent &swerve) {
  gyro_.Link(swerve.swerve_.robot_angle);

  controller_status_.esc_factor_0.Link(bldc[0].factor);
  swerve.swerve_.motors[0]->drive_.Link(bldc[0]);
  swerve.swerve_.motors[0]->steer_.output.Link(
      driving_->GetSwerveRot0().GetMotor());

  controller_status_.esc_factor_1.Link(bldc[1].factor);
  swerve.swerve_.motors[1]->drive_.Link(bldc[1]);
  swerve.swerve_.motors[1]->steer_.output.Link(
      driving_->GetSwerveRot1().GetMotor());

  controller_status_.esc_factor_2.Link(bldc[2].factor);
  swerve.swerve_.motors[2]->drive_.Link(bldc[2]);
  swerve.swerve_.motors[2]->steer_.output.Link(
      driving_->GetSwerveRot2().GetMotor());
}

void Communication::LinkToUpper(korobo2023c::Upper &upper) {
  {
    auto &motor = driving_->GetElevation();
    upper.elevation_motor >> motor.GetMotor();
    upper.elevation_motor_factor >> motor.GetMotor().factor;
  }
  {
    auto &motor = driving_->GetHorizontal();
    upper.rotation_motor >> motor.GetMotor();
    upper.rotation_motor_factor >> motor.GetMotor().factor;
  }
  {
    auto &motor = driving_->GetLoad();
    upper.load >> motor.GetMotor();
  }

  upper.shot_r = std::unique_ptr<robotics::node::Motor<float>>(
      &driving_->GetShotR().GetMotor());
  upper.shot_l = std::unique_ptr<robotics::node::Motor<float>>(
      &driving_->GetShotL().GetMotor());

  upper.revolver = std::unique_ptr<robotics::node::Motor<float>>(
      &driving_->GetRevolver().GetMotor());

  upper.load.Link(driving_->GetLoad().GetMotor());

  upper.LinkController();
}

void Communication::AddCAN1Debug() {
  can_.OnRx([](uint32_t id, std::vector<uint8_t> data) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(4) << id << ": ";
    for (auto byte : data) {
      ss << std::setw(2) << std::hex << (int)byte << " ";
    }
    printf("CAN1<-- %s\n", ss.str().c_str());
  });
  can_.OnTx([](uint32_t id, std::vector<uint8_t> data) {
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(4) << id << ": ";
    for (auto byte : data) {
      ss << std::setw(2) << std::hex << (int)byte << " ";
    }
    printf("CAN1--> %s\n", ss.str().c_str());
  });
}

void Communication::Report() {
  switch (report_counter) {
    case 0:
      ReportBLDC();
      break;
    case 1:
    case 2:
    case 3:
      driving_->ReportTo(can_);
      break;
  }
  report_counter = (report_counter + 1) % 4;
}