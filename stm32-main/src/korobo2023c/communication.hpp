#pragma once

#include "2023c.hpp"
#include "../dcan.hpp"
#include "bus/driving_can.hpp"
#include "../robotics/sensor/gyro/bno055.hpp"
#include "../robotics/node/BLDC.hpp"

#include "components/swerve.hpp"

class Communication {
 public:
  struct Config {
    struct {
      int id;
      int freqency;

      PinName rx, tx;
    } can;

    struct {
      PinName rx, tx;
    } driving_can;

    korobo::n2023c::Controller::Config controller_ids;

    korobo::n2023c::ValueStoreMain<float>::Config value_store_ids;

    struct {
      PinName sda;
      PinName scl;
    } i2c;

    struct {
      PinName swerve_pin_m0;
      PinName swerve_pin_m1;
      PinName swerve_pin_m2;
    } swerve_esc_pins;
  };

 public:
  DistributedCAN can_;
  std::unique_ptr<DrivingCANBus> driving_;

  korobo::n2023c::Controller controller_status_;
  korobo::n2023c::ValueStoreMain<float> value_store_;

  robotics::sensor::gyro::BNO055 gyro_;
  robotics::node::BLDC bldc[3];

 private:
  void InitCAN() {
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing CAN (Com)\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Initializing CAN Driver\n");
    can_.Init();
    driving_->Init();
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Adding Handlers\n");
    can_.OnEvent(0x40, [this](std::vector<uint8_t> data) {  //
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
    can_.OnEvent(0x61, [this](std::vector<uint8_t> data) {  //
      value_store_.Pass(data);
    });
  }

  void InitGyro() {
    SetStatus(DistributedCAN::Statuses::kInitializingGyro);
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing Gyro\n");
    auto gyro_init_status = gyro_.Init();
    if (!gyro_init_status) {
      printf(
          "\e[1;32m|\e[m \e[32m-\e[m \e[1;31m=> !!!\e[m BNO055 Detection "
          "failed.\e[m\n");
    }
  }

  void InitBLDC() {
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing ESC\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Max Pulsewidth\n");
    SetStatus(DistributedCAN::Statuses::kInitializingESC0);
    bldc[0].Init0();
    bldc[1].Init0();
    bldc[2].Init0();
    ThisThread::sleep_for(2s);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Min Pulsewidth\n");
    SetStatus(DistributedCAN::Statuses::kInitializingESC1);
    bldc[0].Init1();
    bldc[1].Init1();
    bldc[2].Init1();
    ThisThread::sleep_for(1s);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m OK\n");
  }

 public:
  Communication(Config &config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        driving_(std::make_unique<DrivingCANBus>(new ikarashiCAN_mk2(
            config.driving_can.rx, config.driving_can.tx, 0))),
        controller_status_(config.controller_ids),
        value_store_(config.value_store_ids),
        gyro_(config.i2c.sda, config.i2c.scl),
        bldc{
            {config.swerve_esc_pins.swerve_pin_m0, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m1, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m2, 1000, 2000},
        } {}

  void SendNonReactiveValues() {
    this->driving_->Tick();
    this->driving_->Send();
  }

  void Init() {
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

  void SetStatus(DistributedCAN::Statuses status) { can_.SetStatus(status); }

  void LinkToSwerve(SwerveComponent &swerve) {
    gyro_.Link(swerve.swerve_.robot_angle);

    swerve.swerve_.motors[0]->drive_.Link(bldc[0]);
    swerve.swerve_.motors[0]->steer_.output.Link(
        driving_->GetSwerveRot0().GetMotor());

    swerve.swerve_.motors[1]->drive_.Link(bldc[1]);
    swerve.swerve_.motors[1]->steer_.output.Link(
        driving_->GetSwerveRot1().GetMotor());

    swerve.swerve_.motors[2]->drive_.Link(bldc[2]);
    swerve.swerve_.motors[2]->steer_.output.Link(
        driving_->GetSwerveRot2().GetMotor());
  }
};