#pragma once

#include "mbed.h"
#include "rtos.h"

class DistributedCAN {
  int can_id = 0x7;
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
      WriteMessage(0x81, {});
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
        ThisThread::sleep_for(10ms);
      }

      if (can_.read(msg)) {
        HandleMessage(msg);
      }
    }
  }

 public:
  DistributedCAN(int can_id, PinName rx, PinName tx, int freqency = 50E3)
      : can_id(can_id), can_(rx, tx, freqency), freqency_(freqency) {}

  void Init() { thread_.start(callback(this, &DistributedCAN::ThreadMain)); }

  // 1 -> success
  int Send(uint32_t id, std::vector<uint8_t> const &data) {
    return WriteMessage(id, data);
  }
};