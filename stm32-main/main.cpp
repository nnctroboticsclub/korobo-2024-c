#include "mbed.h"
#include "rtos.h"

using namespace std::chrono_literals;

Ticker ticker;
DigitalOut led1(LED1);
CAN can(PB_8, PB_9, 1000000);

char counter = 0;
void ReceiveLoop() {
  CANMessage msg;
  while (1) {
    if (can.read(msg)) {
      printf("Message received ID=0x%#x: ", msg.id);
      for (int i = 0; i < msg.len; i++) {
        printf("%02x ", msg.data[i]);
      }
      printf("\n");
    }

    ThisThread::sleep_for(1ms);
  }
}
void SendLoop() {
  int i = 0;

  while (1) {
    CANMessage msg(0x0, "\0", 1, CANData, CANStandard);
    msg.data[0] = i;
    auto r = can.write(msg);
    if (r == 0) {
      printf("failed to send message\n");
    }
    i++;
    ThisThread::sleep_for(1000ms);
  }
}

int main() {
  printf("main()\n");

  Thread t;
  t.start(ReceiveLoop);

  Thread t2;
  t2.start(SendLoop);

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(500ms);
  }
}
