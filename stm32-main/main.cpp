#include "mbed.h"
#include "rtos.h"

#define wait_ms(x) wait_ns(x * 1000000)

#include "BNO055.h"

#include <vector>

#include "identify.h"
#include "./dcan.hpp"

using namespace std::chrono_literals;

using namespace rtos;

DigitalOut led1(LED1);
DistributedCAN can(CAN_ID, PB_8, PB_9, 1E6);

int main(int argc, char const *argv[]) {
  printf("main() started\n");
  can.Init();

  return 0;
}