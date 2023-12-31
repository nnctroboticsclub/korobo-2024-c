#include "mbed.h"
#include "rtos.h"

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "bno055.hpp"

using namespace std::chrono_literals;

using namespace rtos;

DistributedCAN can(CAN_ID, PB_8, PB_9, 1E6);
Gyro gyro;

int main(int argc, char const *argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  // can.Init();
  gyro.Init(true);

  return 0;
}