#include "mbed.h"
#include "rtos.h"

#include <vector>

using namespace std::chrono_literals;

using namespace rtos;

DigitalOut led1(LED1);

class KoroboCANDriver {
  CAN can_;

  Thread thread_;

  inline void WriteMessage(uint32_t id, std::vector<uint8_t> const &data) {
    CANMessage msg;
    msg.id = id;
    msg.len = data.size();
    std::copy(data.begin(), data.end(), msg.data);
    can_.write(msg);
  }

  inline void HandleMessage(CANMessage const &message) {
    if (message.id == 0x80)  // global ping
    {
      WriteMessage(0x81, {0x02});
    } else if (message.id == 0x40)  // Control packet
    {
      if (message.data[0] == 0x02) {
        WriteMessage(0x82, {0x02});
      }
    }
  }

  void ThreadMain() {
    while (1) {
      if (can_.rderror() || can_.tderror()) {
        can_.reset();

        printf("E rd=%d td=%d\n", can_.rderror(), can_.tderror());

        ThisThread::sleep_for(50ms);
      }

      CANMessage msg;
      if (can_.read(msg)) {
        HandleMessage(msg);
      }
    }
  }

 public:
  KoroboCANDriver(PinName rx, PinName tx, int freqency = 50E3)
      : can_(rx, tx, freqency) {}

  void Init() {
    can_.reset();
    can_.frequency(25E3);

    can_.mode(CAN::Silent);

    if (thread_.get_state() == Thread::Running) {
      thread_.terminate();
    }

    thread_.start(callback(this, &KoroboCANDriver::ThreadMain));
  }
};

int main(int argc, char const *argv[]) {
  printf("main() started\n");
  /* KoroboCANDriver can(PB_8, PB_9);


  can.Init();

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(500ms);
  } */

  return 0;
}