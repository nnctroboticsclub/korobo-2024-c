#include <mbed.h>
#include "main.hpp"

int main(int argc, char const *argv[]) {
  main_switch();
  /* DigitalOut out(PA_4);
  while (1) {
    out = 1;
    ThisThread::sleep_for(100ms);
    out = 0;
    ThisThread::sleep_for(100ms);
  } */
  return 0;
}
