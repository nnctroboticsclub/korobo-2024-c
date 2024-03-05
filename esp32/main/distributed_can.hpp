#pragma once

#include "can_driver.hpp"

constexpr const uint8_t kDeviceId = 2;

class KoroboCANDriver {
 public:
  using PongListener = std::function<void(uint8_t device)>;

 private:
  CANDriver can_;
  std::vector<PongListener> pong_listeners_;

 public:
  KoroboCANDriver() : can_() {}

  void Init(gpio_num_t tx, gpio_num_t rx) {
    can_.Init(tx, rx);

    can_.OnMessage(0x80, [this](uint32_t id, std::vector<uint8_t> const& data) {
      can_.SendStd(0x81 + kDeviceId, {kDeviceId});
    });

    for (uint8_t device = 0; device < 15; device++) {
      can_.OnMessage(
          0x81 + device,
          [this, device](uint32_t id, std::vector<uint8_t> const& data) {
            for (auto& cb : pong_listeners_) {
              cb(device);
            }
          });
    }
  }

  void SendStd(uint32_t id, std::vector<uint8_t> const& data) {
    can_.SendStd(id, data);
  }

  void OnMessage(uint32_t id, CANDriver::Callback cb) {
    can_.OnMessage(id, cb);
  }

  void OnRx(CANDriver::Callback cb) { can_.OnRx(cb); }
  void OnTx(CANDriver::Callback cb) { can_.OnTx(cb); }

  float GetBusLoad() { return can_.GetBusLoad(); }

  void SendControl(std::vector<uint8_t> const& data) { SendStd(0x40, data); }

  void Ping() { SendStd(0x80, {}); }

  void OnPong(PongListener cb) { pong_listeners_.emplace_back(cb); }
};