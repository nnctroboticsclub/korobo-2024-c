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
    }
  }

  void ThreadMain() {
    while (1) {
      if (can_.rderror() || can_.tderror()) {
        printf("E\n");
        can_.reset();
        ThisThread::sleep_for(10ms);
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
    can_.frequency(50E3);
    can_.mode(CAN::Normal);

    if (thread_.get_state() == Thread::Running) {
      thread_.terminate();
    }

    thread_.start(callback(this, &KoroboCANDriver::ThreadMain));
  }
};

int main(int argc, char const *argv[]) {
  KoroboCANDriver can(PB_8, PB_9);

  can.Init();

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(500ms);
  }

  return 0;
}