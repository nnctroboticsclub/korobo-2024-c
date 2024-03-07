#pragma once

#include "mbed.h"
#include "rtos.h"

#include <unordered_map>
#include <functional>
#include <vector>
#include <sstream>
#include <iomanip>

class SimpleCAN {
 public:
  using RxCallback =
      std::function<void(std::uint32_t, std::vector<uint8_t> const &)>;
  using TxCallback =
      std::function<void(std::uint32_t, std::vector<uint8_t> const &)>;

 private:
  CAN can_;
  int freqency_ = 50E3;

  std::vector<RxCallback> rx_callbacks_;
  std::vector<TxCallback> tx_callbacks_;

  Thread *thread_;

  void ThreadMain() {
    CANMessage msg;
    while (1) {
      if (can_.rderror() || can_.tderror()) {
        can_.reset();
        ThisThread::sleep_for(10ms);
      }

      if (can_.read(msg)) {
        for (auto const &cb : rx_callbacks_) {
          cb(msg.id, std::vector<uint8_t>(msg.data, msg.data + msg.len));
        }
      }
    }
  }

 public:
  // 1 -> success
  inline int Send(uint32_t id, std::vector<uint8_t> const &data) {
    for (auto const &cb : tx_callbacks_) {
      cb(id, data);
    }

    CANMessage msg;
    msg.id = id;
    msg.len = data.size();
    std::copy(data.begin(), data.end(), msg.data);
    return can_.write(msg);
  }

  SimpleCAN(PinName rx, PinName tx, int freqency = 50E3)
      : can_(rx, tx, freqency), freqency_(freqency) {}

  void Init() {
    thread_ = new Thread(osPriorityNormal, 1024 * 4);
    thread_->start(callback(this, &SimpleCAN::ThreadMain));
  }

  void OnRx(RxCallback cb) { rx_callbacks_.emplace_back(cb); }

  void OnTx(TxCallback cb) { tx_callbacks_.emplace_back(cb); }
};

class DistributedCAN {
 public:
  enum class Statuses {
    kReady = 0x00,
    kCANReady = 0x01,
    kInitializingESC = 0x01,
    kInitializingGyro = 0x02,
  };

  struct EventCallback {
    uint8_t element_id;
    std::function<void(std::vector<uint8_t>)> cb;
  };

 private:
  int can_id = 0x7;
  SimpleCAN can_;

  std::vector<EventCallback> callbacks_;

  inline void HandleMessage(uint32_t id, std::vector<uint8_t> const &data) {
    for (auto const &cb : callbacks_) {
      if (cb.element_id == id) {
        cb.cb(data);
      }
    }
  }

 public:
  DistributedCAN(int can_id, PinName rx, PinName tx, int freqency = 50E3)
      : can_id(can_id), can_(rx, tx, freqency) {}

  void Init() {
    can_.Init();
    can_.OnRx([this](uint32_t id, std::vector<uint8_t> const &data) {
      this->HandleMessage(id, data);
    });

    OnEvent(0x80, [this](std::vector<uint8_t> data) {
      can_.Send(0x81 + can_id, {});
    });

    SetStatus(Statuses::kCANReady);
  }

  void OnEvent(uint8_t element_id,
               std::function<void(std::vector<uint8_t>)> cb) {
    callbacks_.emplace_back(EventCallback{element_id, cb});
  }

  void OnRx(SimpleCAN::RxCallback cb) { can_.OnRx(cb); }
  void OnTx(SimpleCAN::TxCallback cb) { can_.OnTx(cb); }

  int Send(uint8_t element_id, std::vector<uint8_t> const &data) {
    return can_.Send(element_id, data);
  }

  void SetStatus(Statuses status) {
    std::vector<uint8_t> payload(4);
    payload[0] = 0xf0;
    payload[1] = (can_id >> 0x08) && 0xff;
    payload[2] = (can_id >> 0x00) & 0xff;
    payload[3] = static_cast<uint8_t>(status);

    can_.Send(0xa0, payload);
  }
};