#include <mbed.h>
#include <rtos.h>
#include <ikakoMDC.h>

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "controller/korobo/2023c.hpp"

#include "robotics/component/swerve/swerve.hpp"
#include "robotics/sensor/gyro/bno055.hpp"
#include "robotics/output/ikakoMDC.hpp"

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
      PinName sda, scl;
    } bno055;

    controller::Korobo2023Controller::Config controller_ids;
    controller::Korobo2023MainValueStore::Config value_store_ids;
    robotics::component::Swerve::Config&& swerve;
  };

 private:
  DistributedCAN can_;
  robotics::sensor::gyro::Gyro gyro_;
  robotics::component::Swerve swerve_;

  controller::Korobo2023Controller controller_status_;
  controller::Korobo2023MainValueStore value_store;

  Thread monitor_thread_;
  Thread robo_thread_;

  void MonitorThread() {
    while (1) {
      // printf("steer: m(%+3d %+3d)r(%d %d p%d)\n",
      //        controller_status_.steer_move.x,
      //        controller_status_.steer_move.y,
      //        controller_status_.steer_angle.magnitude_,
      //        controller_status_.steer_angle.angle_,
      //        controller_status_.steer_rotation_pid_enabled.value_);
      // printf("v: steer: %lf %lf %lf, d: %lf\n",
      //        value_store.steer_motor_0_encoder.value_,
      //        value_store.steer_motor_1_encoder.value_,
      //        value_store.steer_motor_2_encoder.value_,
      //        value_store.dummy_encoder.value_);
      printf("g: v:(%lf %lf)\n", gyro_.GetHorizontalOrientation());
      ThisThread::sleep_for(100ms);
    }
  }

  void RoboThread() {
    //* Steering
  }

 public:
  App(Config& config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        gyro_(config.bno055.sda, config.bno055.scl),
        controller_status_(config.controller_ids),
        value_store(config.value_store_ids),
        swerve_(config.swerve) {}

  void Init() {
    can_.Init();
    gyro_.Init(false);

    can_.OnEvent(0x40, [this](std::vector<uint8_t> data) {
      controller_status_.Parse(data);
    });
    can_.OnEvent(
        0x61, [this](std::vector<uint8_t> data) { value_store.Parse(data); });
  }

  void Main() {
    monitor_thread_.start(callback(this, &App::MonitorThread));
    robo_thread_.start(callback(this, &App::RoboThread));

    while (1) {
      ThisThread::sleep_for(1s);
    }
  }
};

int main(int argc, char const* argv[]) {
  using namespace robotics::component::swerve;
  using namespace robotics;
  printf("main() started CAN_ID=%d\n", CAN_ID);

  Motor motor1{
      .drive = 0,
      .steer = std::make_unique<fusion::AngledMotor<float>>(
          fusion::AngledMotor<float>::Config{
              .encoder = std::make_unique<sensor::encoder::Absolute<float>>(
                  PB_0, PB_1),
              .motor = std::make_unique<output::ikakoMDCMotor>(0, -50, 50),
          }),
      .angle_deg = 0,
  };

  Motor motor2{
      .drive = std::make_unique<output::Motor<float>>(PA_2, PA_3),
      .steer = std::make_unique<fusion::AngledMotor<float>>(
          fusion::AngledMotor<float>::Config{
              .encoder = std::make_unique<sensor::encoder::Absolute<float>>(
                  PB_2, PB_3),
              .motor = std::make_unique<output::ikakoMDCMotor>(1, -50, 50),
          }),
      .angle_deg = 120,
  };

  Motor motor3{
      .drive = std::make_unique<output::Motor<float>>(PA_4, PA_5),
      .steer = std::make_unique<fusion::AngledMotor<float>>(
          fusion::AngledMotor<float>::Config{
              .encoder = std::make_unique<sensor::encoder::Absolute<float>>(
                  PB_10, PB_11),
              .motor = std::make_unique<output::Motor<float>>(PB_12, PB_13),
          }),
      .angle_deg = 240,
  };

  Swerve::Config swerve_config{
      .motors =
          {
              motor1,
              motor2,
              motor3,
          },
      .gyro = std::make_shared<robotics::sensor::gyro::BNO055>(PC_9, PA_8),
  };

  App::Config config{.can =
                         {
                             .id = CAN_ID,
                             .freqency = (int)1E6,
                             .rx = PB_8,
                             .tx = PB_9,
                         },
                     .bno055 =
                         {
                             .sda = PC_9,
                             .scl = PA_8,
                         },
                     .swerve = {.}};

  App app(config);
  app.Init();
  app.Main();

  return 0;
}