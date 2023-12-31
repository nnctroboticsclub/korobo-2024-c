#include "mbed.h"
#include "rtos.h"
#include "bno055.h"

#include <vector>

#include "identify.h"
#include "./dcan.hpp"

using namespace std::chrono_literals;

using namespace rtos;

DigitalOut led1(LED1);
DistributedCAN can(CAN_ID, PB_8, PB_9, 1E6);

class Gyro {
  BNO055 bno055_;
  Thread thread;

  void ThreadMain() {
    if (!bno055_.check()) {
      printf("BNO055 not detected\n");
      return;
    }

    bno055_.reset();

    bno055_.setpowermode(POWER_MODE_NORMAL);
    bno055_.setmode(OPERATION_MODE_ACCGYRO);

    while (true) {
      ThisThread::sleep_for(100ms);
    }
  }

 public:
  Gyro() : bno055_(PC_9, PA_8), thread() {
    thread.start(callback(this, &Gyro::ThreadMain));
  }
};

int main(int argc, char const *argv[]) {
  printf("main() started\n");
  can.Init();

  return 0;
}