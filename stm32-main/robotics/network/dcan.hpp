#pragma once

#include <unordered_map>
#include <functional>
#include <vector>
#include <sstream>
#include <iomanip>

#include "simple_can.hpp"

class DistributedCAN {
 public:
  enum class Statuses : uint8_t {
    kReady = 0x00,
    kCANReady = 0x01,
    kInitializingESC0 = 0x02,
    kInitializingESC1 = 0x03,
    kInitializingGyro = 0x04,
  };

  struct EventCallback {
    uint8_t element_id;
    std::function<void(std::vector<uint8_t>)> cb;
  };

  using KeepAliveLostCallback = std::function<void()>;
  using KeepAliveRecoverdCallback = std::function<void()>;

 private:
  int can_id = 0x7;
  SimpleCAN can_;
  int last_keep_alive = 0;
  Timer keep_alive_timer;

  std::vector<EventCallback> callbacks_;

  bool keep_alive_available;
  std::vector<KeepAliveLostCallback> keep_alive_lost_callbacks_;
  std::vector<KeepAliveRecoverdCallback> keep_alive_recovered_callbacks_;

  inline void HandleMessage(uint32_t id, std::vector<uint8_t> const &data);

 public:
  DistributedCAN(int can_id, PinName rx, PinName tx, int freqency = 50E3);

  void Init();

  void OnMessage(uint8_t element_id,
                 std::function<void(std::vector<uint8_t>)> cb);

  void OnRx(SimpleCAN::RxCallback cb);
  void OnTx(SimpleCAN::TxCallback cb);
  void OnIdle(SimpleCAN::IdleCallback cb);
  void OnKeepAliveLost(KeepAliveLostCallback cb);
  void OnKeepAliveRecovered(KeepAliveRecoverdCallback cb);

  int Send(uint8_t element_id, std::vector<uint8_t> const &data);
  void SetStatus(Statuses status);
};