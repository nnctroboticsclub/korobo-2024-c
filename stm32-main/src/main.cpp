#include <mbed.h>
#include <rtos.h>

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "korobo2023c/2023c.hpp"

#include "robotics/component/swerve/swerve.hpp"
#include "robotics/sensor/gyro/bno055.hpp"
#include "robotics/node/ikakoMDC.hpp"
#include "robotics/node/BLDC.hpp"
#include "robotics/types/angle_joystick_2d.hpp"

using namespace std::chrono_literals;

using namespace rtos;

using robotics::filter::PID;

class App {
 public:
  struct Config {
    struct {
      int id;
      int freqency;

      PinName rx, tx;
    } can;

    struct {
      PinName sda;
      PinName scl;
    } i2c;

    struct {
      PinName swerve_pin_m0;
      PinName swerve_pin_m1;
      PinName swerve_pin_m2;
    } swerve_esc_pins;

    robotics::component::Swerve::Config swerve_config;
    korobo::n2023c::Controller::Config controller_ids;
    korobo::n2023c::ValueStoreMain<float>::Config value_store_ids;
  };

 private:
 private:
  DistributedCAN can_;
  robotics::sensor::gyro::BNO055 gyro_;
  korobo::n2023c::Controller controller_status_;
  korobo::n2023c::ValueStoreMain<float> value_store_;
  robotics::component::Swerve swerve_;
  robotics::node::BLDC bldc[3];
  robotics::node::ikakoMDCMotor motors[3];

  Thread monitor_thread_;
  Thread robo_thread_;

  void MonitorThread() {
    while (1) {
      ThisThread::sleep_for(100ms);
    }
  }

  void RoboThread() {
    //* Steering
  }

 public:
  App(Config &config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        gyro_(config.i2c.sda, config.i2c.scl),
        controller_status_(config.controller_ids),
        value_store_(config.value_store_ids),
        swerve_(config.swerve_config),
        bldc{
            {config.swerve_esc_pins.swerve_pin_m0, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m1, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m2, 1000, 2000},
        },
        motors{
            {0, -50, 50},
            {1, -50, 50},
            {2, -50, 50},
        } {
    controller_status_.swerve.angle.SetChangeCallback(
        [this](robotics::AngleStick2D angle) {
          swerve_.angle_ctrl.SetValue(angle.angle);
        });

    controller_status_.swerve.angle_pid.Link(swerve_.angle.gains);
    controller_status_.swerve.motor_0_pid.Link(
        swerve_.motors[0]->steer_.pid.gains);
    controller_status_.swerve.motor_1_pid.Link(
        swerve_.motors[1]->steer_.pid.gains);
    controller_status_.swerve.motor_2_pid.Link(
        swerve_.motors[2]->steer_.pid.gains);

    controller_status_.swerve.move.Link(swerve_.move_ctrl);

    // TODO: Link rotation_pid_enabled
    // Muxer 導入？
    // controller_status_.swerve.rotation_pid_enabled;

    // swerve out link
    swerve_.motors[0]->drive_.Link(bldc[0]);
    swerve_.motors[0]->steer_.output.Link(motors[0]);

    swerve_.motors[1]->drive_.Link(bldc[1]);
    swerve_.motors[1]->steer_.output.Link(motors[1]);

    swerve_.motors[2]->drive_.Link(bldc[2]);
    swerve_.motors[2]->steer_.output.Link(motors[2]);
  }

  void Init() {
    can_.Init();

    can_.OnEvent(0x40, [this](std::vector<uint8_t> data) {
      controller_status_.Parse(data);
    });
    can_.OnEvent(
        0x61, [this](std::vector<uint8_t> data) { value_store_.Parse(data); });

    can_.SetStatus(DistributedCAN::Statuses::kInitializingESC);
  }

  void Main() {
    monitor_thread_.start(callback(this, &App::MonitorThread));
    robo_thread_.start(callback(this, &App::RoboThread));

    while (1) {
      swerve_.Update(0.01f);
      ThisThread::sleep_for(10s);
    }
  }
};

using namespace robotics::component;

int main(int argc, char const *argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  Swerve::Config swerve_config = {.angle_offsets = {0, 120, 240}};

  App::Config config{
      .can =
          {
              .id = CAN_ID,
              .freqency = (int)1E6,
              .rx = PB_8,
              .tx = PB_9,
          },
      .i2c =
          {
              .sda = PB_7,
              .scl = PB_6,
          },
      .swerve_esc_pins =
          {
              .swerve_pin_m0 = PA_0,
              .swerve_pin_m1 = PA_1,
              .swerve_pin_m2 = PA_2,
          },
      .swerve_config = swerve_config,
      .controller_ids = {.swerve =
                             (controller::swerve::SwerveController::Config){
                                 .joystick_id = 0,
                                 .angle_joystick_id = 1,
                                 .rotation_pid_enabled_id = 1,
                                 .motor_0_pid_id = 0,
                                 .motor_1_pid_id = 1,
                                 .motor_2_pid_id = 2,
                                 .angle_pid_id = 3,
                             }},
      .value_store_ids =
          {.swerve =
               (controller::swerve::SwerveValueStore<float>::Config){
                   .motor_0_encoder_id = 0,
                   .motor_1_encoder_id = 1,
                   .motor_2_encoder_id = 2}},
  };

  App app(config);
  app.Init();
  app.Main();

  return 0;
}