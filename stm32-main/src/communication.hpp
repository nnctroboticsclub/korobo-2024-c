#pragma once

#include <robotics/network/dcan.hpp>
#include <robotics/node/BLDC.hpp>
#include <mbed-robotics/bno055.hpp>

#include "2023c.hpp"
#include "bus/driving_can.hpp"

#include "components/swerve.hpp"
#include "components/upper.hpp"

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

    korobo2023c::Controller::Config controller_ids;

    korobo2023c::ValueStoreMain<float>::Config value_store_ids;

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
  robotics::network::DistributedCAN can_;
  std::unique_ptr<DrivingCANBus> driving_;

  korobo2023c::Controller controller_status_;
  korobo2023c::ValueStoreMain<float> value_store_;

  robotics::sensor::gyro::BNO055 gyro_;
  robotics::node::BLDC bldc[3];

  int report_counter;

 private:
  void InitCAN();
  void InitGyro();
  void InitBLDC();
  void ReportBLDC();

 public:
  Communication(Config &config);

  void SendNonReactiveValues();
  void Init();
  void SetStatus(robotics::network::DistributedCAN::Statuses status);
  void LinkToSwerve(SwerveComponent &swerve);
  void LinkToUpper(korobo2023c::Upper &upper);
  void AddCAN1Debug();
  void Report();
};
