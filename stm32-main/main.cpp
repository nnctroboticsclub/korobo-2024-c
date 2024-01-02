#include "mbed.h"
#include "rtos.h"

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "bno055.hpp"
#include "controller.hpp"

using namespace std::chrono_literals;

using namespace rtos;

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
  Gyro gyro_;
  controller::ControllerStatus controller_status_;

  Thread monitor_thread_;

  void MonitorThread() {
    while (1) {
      printf("steer: m(%+3d %+3d)r(%d %d)\n", controller_status_.steer_move.x,
             controller_status_.steer_move.y, controller_status_.steer_angle.x,
             controller_status_.steer_angle.y);
      ThisThread::sleep_for(50ms);
    }
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
  }

  void Main() {
    monitor_thread_.start(callback(this, &App::MonitorThread));

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