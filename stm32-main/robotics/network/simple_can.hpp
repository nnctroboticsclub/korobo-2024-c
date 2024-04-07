#pragma once

#include <functional>
#include <mbed.h>
#include <rtos.h>

class SimpleCAN {
 public:
  using RxCallback =
      std::function<void(std::uint32_t, std::vector<uint8_t> const &)>;
  using TxCallback =
      std::function<void(std::uint32_t, std::vector<uint8_t> const &)>;
  using IdleCallback = std::function<void()>;

 private:
  CAN can_;
  int freqency_ = 50E3;

  std::vector<RxCallback> rx_callbacks_;
  std::vector<TxCallback> tx_callbacks_;
  std::vector<IdleCallback> idle_callbacks_;

  Thread *thread_;

  void ThreadMain();

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

  SimpleCAN(PinName rx, PinName tx, int freqency = 50E3);

  void Init();

  void OnRx(RxCallback cb);
  void OnTx(TxCallback cb);
  void OnIdle(IdleCallback cb);
};