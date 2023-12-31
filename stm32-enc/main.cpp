#include "mbed.h"
#include "rtos.h"

#include <vector>

using namespace std::chrono_literals;

using namespace rtos;

DigitalOut led1(LED1);

class KoroboCANDriver {
  CAN can_;
  int freqency_ = 50E3;

  Thread thread_;

  // 1 -> success
  inline int WriteMessage(uint32_t id, std::vector<uint8_t> const &data) {
    CANMessage msg;
    msg.id = id;
    msg.len = data.size();
    std::copy(data.begin(), data.end(), msg.data);
    return can_.write(msg);
  }

  inline void HandleMessage(CANMessage const &message) {
    if (message.id == 0x80)  // global ping
    {
      WriteMessage(0x82, {});
    } else if (message.id == 0x40)  // Control packet
    {
      if (message.data[0] == 0x02) {
        WriteMessage(0x82, {0x02});
      }
    } else {
      printf("Message received from %#lx: ", message.id);
      for (int i = 0; i < message.len; i++) {
        printf("%02x ", message.data[i]);
      }
      printf("\n");
    }
  }

  void ThreadMain() {
    CANMessage msg;
    while (1) {
      if (can_.rderror() || can_.tderror()) {
        can_.reset();

        // printf("E rd=%d td=%d\n", can_.rderror(), can_.tderror());

        ThisThread::sleep_for(10ms);
      }

      if (can_.read(msg)) {
        HandleMessage(msg);
      }
    }
  }

 public:
  KoroboCANDriver(PinName rx, PinName tx, int freqency = 50E3)
      : can_(rx, tx, freqency), freqency_(freqency) {}

  void Init() { thread_.start(callback(this, &KoroboCANDriver::ThreadMain)); }

  // 1 -> success
  int Send(uint32_t id, std::vector<uint8_t> const &data) {
    return WriteMessage(id, data);
  }
};

int main(int argc, char const *argv[]) {
  printf("main() started\n");
  KoroboCANDriver can(PB_8, PB_9, 1E6);

  can.Init();

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(500ms);

    if (led1) {
      auto status = can.Send(0xa0, {0x01, 0x02, 0x03, 0x04, 0x05});
      if (status != 1) {
        printf("Failed to send message: %d\n", status);
      }
    }
  }

  return 0;
}