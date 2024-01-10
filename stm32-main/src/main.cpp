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
  App(Config const& config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        gyro_(config.bno055.sda, config.bno055.scl) {}

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
  printf("main() started CAN_ID=%d\n", CAN_ID);

  App::Config config{
      .can =
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
  };

  App app(config);
  app.Init();
  app.Main();

  return 0;
}