#include "dcan.hpp"

void SimpleCAN::ThreadMain() {
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

    if (!this->idle_callbacks_.empty()) {
      for (auto cb : this->idle_callbacks_) {
        cb();
      }
    }
  }
}

inline int SimpleCAN::Send(uint32_t id, std::vector<uint8_t> const &data) {
  for (auto const &cb : tx_callbacks_) {
    cb(id, data);
  }

  CANMessage msg;
  msg.id = id;
  msg.len = data.size();
  std::copy(data.begin(), data.end(), msg.data);
  return can_.write(msg);
}

SimpleCAN::SimpleCAN(PinName rx, PinName tx, int freqency)
    : can_(rx, tx, freqency), freqency_(freqency) {}

void SimpleCAN::Init() {
  thread_ = new Thread(osPriorityNormal, 1024 * 4);
  thread_->start(callback(this, &SimpleCAN::ThreadMain));
}

void SimpleCAN::OnRx(RxCallback cb) { rx_callbacks_.emplace_back(cb); }

void SimpleCAN::OnTx(TxCallback cb) { tx_callbacks_.emplace_back(cb); }

void SimpleCAN::OnIdle(IdleCallback cb) { idle_callbacks_.emplace_back(cb); }

inline void DistributedCAN::HandleMessage(uint32_t id,
                                          std::vector<uint8_t> const &data) {
  for (auto const &cb : callbacks_) {
    if (cb.element_id == id) {
      cb.cb(data);
    }
  }
}

DistributedCAN::DistributedCAN(int can_id, PinName rx, PinName tx, int freqency)
    : can_id(can_id), can_(rx, tx, freqency) {
  keep_alive_timer.start();
}

void DistributedCAN::Init() {
  can_.Init();
  can_.OnRx([this](uint32_t id, std::vector<uint8_t> const &data) {
    this->HandleMessage(id, data);
  });

  OnEvent(0x80,
          [this](std::vector<uint8_t> data) { can_.Send(0x81 + can_id, {}); });

  OnEvent(0xfc, [this](std::vector<uint8_t> _) {
    // printf("Keepalive!\n");
    keep_alive_timer.reset();
  });
  can_.OnIdle([this]() {
    auto timer = keep_alive_timer.read_ms();
    if (keep_alive_available && 300 < timer) {
      printf("Keepalive Lost!\n");
      keep_alive_available = false;
      for (auto &&cb : this->keep_alive_lost_callbacks_) {
        cb();
      }
    } else if (!keep_alive_available && timer < 300) {
      printf("Keepalive Get!\n");
      keep_alive_available = true;
      for (auto &&cb : this->keep_alive_recovered_callbacks_) {
        cb();
      }
    }
  });

  SetStatus(Statuses::kCANReady);
}

void DistributedCAN::OnEvent(uint8_t element_id,
                             std::function<void(std::vector<uint8_t>)> cb) {
  callbacks_.emplace_back(EventCallback{element_id, cb});
}

void DistributedCAN::OnRx(SimpleCAN::RxCallback cb) { can_.OnRx(cb); }
void DistributedCAN::OnTx(SimpleCAN::TxCallback cb) { can_.OnTx(cb); }
void DistributedCAN::OnIdle(SimpleCAN::IdleCallback cb) { can_.OnIdle(cb); }
void DistributedCAN::OnKeepAliveLost(KeepAliveLostCallback cb) {
  this->keep_alive_lost_callbacks_.emplace_back(cb);
}
void DistributedCAN::OnKeepAliveRecovered(KeepAliveRecoverdCallback cb) {
  this->keep_alive_recovered_callbacks_.emplace_back(cb);
}

int DistributedCAN::Send(uint8_t element_id, std::vector<uint8_t> const &data) {
  return can_.Send(element_id, data);
}

void DistributedCAN::SetStatus(Statuses status) {
  std::vector<uint8_t> payload(4);
  payload[0] = 0xf0;
  payload[1] = (can_id >> 0x08) && 0xff;
  payload[2] = (can_id >> 0x00) & 0xff;
  payload[3] = static_cast<uint8_t>(status);

  can_.Send(0xa0, payload);
}