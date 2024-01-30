#include "mbed.h"
#include "rtos.h"
#include "QEI.h"

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"

using namespace std::chrono_literals;

using namespace rtos;

void AtomicPrint(std::string const& string) {
  static Mutex* mtx = nullptr;
  if (!mtx) mtx = new Mutex;

  mtx->lock();
  printf("%s", string.c_str());
  mtx->unlock();
}

template <typename T>
class UpdateDetector {
 private:
  T value_;
  bool updated_;
  bool invalidated_ = true;

 public:
  UpdateDetector() : value_(0), updated_(false) {}
  UpdateDetector(T const& value) : value_(value), updated_(false) {}

  bool Update(T const& value) {
    if (invalidated_) {
      invalidated_ = false;
      value_ = value;
      updated_ = true;
    } else if (value != value_) {
      value_ = value;
      updated_ = true;
    } else {
      updated_ = false;
    }
    return updated_;
  }
};

class PseudoAbsEncoder {
 private:
  int pulses_per_rotation = 200;

  int current_pulses = 0;
  bool is_abs_ready = false;

  InterruptIn A_;
  InterruptIn B_;
  InterruptIn index_;

 public:
  struct Config {
    PinName a, b, index;
  };

  PseudoAbsEncoder(PinName A, PinName B, PinName index)
      : A_(A), B_(B), index_(index) {
    printf("PseudoAbsEncoder(%d, %d, %d)\n", A, B, index);

    A_.rise([this]() {
      auto b = B_.read();
      if (b) {
        current_pulses += 1;
      } else {
        current_pulses -= 1;
      }
    });
    A_.fall([this]() {
      auto b = B_.read();
      if (b) {
        current_pulses -= 1;
      } else {
        current_pulses += 1;
      }
    });
    index_.rise([this]() {
      current_pulses = 0;
      is_abs_ready = true;
    });

    printf("PseudoAbsEncoder(%d, %d, %d) done\n", A, B, index);
  }

  PseudoAbsEncoder(Config const& config)
      : PseudoAbsEncoder(config.a, config.b, config.index) {}

  double GetAngle() { return 360 * current_pulses / pulses_per_rotation; }
  int GetPulses() { return current_pulses; }

  bool IsAbsReady() { return is_abs_ready; }
};

class DistributedPseudoAbsEncoder : public PseudoAbsEncoder {
 private:
  UpdateDetector<double> angle_;
  UpdateDetector<bool> abs_ready_detector_;

  struct {
    bool enabled;
    DistributedCAN* can;
    int dev_id;
  } attached_can_;

  void Send_() {
    if (!attached_can_.enabled) return;

    uint16_t value = (int16_t)(this->GetAngle() / 360.0f * 0x7FFF);

    std::vector<uint8_t> payload;
    payload.push_back(0x60 | attached_can_.dev_id);
    payload.push_back(value >> 8);
    payload.push_back(value & 0xFF);

    auto ret = attached_can_.can->Send(0x61, payload);
    if (ret != 1) {
      printf("DistributedPseudoAbsEncoder::Send_/CanSend() failed\n");
    }
  }
  void SendAbsReady_() {
    if (!attached_can_.enabled) return;

    std::vector<uint8_t> payload;
    payload.push_back(0xe0);
    payload.push_back(attached_can_.dev_id);
    payload.push_back(this->IsAbsReady());
    payload.push_back(0x00);

    auto ret = attached_can_.can->Send(0x61, payload);
    if (ret != 1) {
      printf("DistributedPseudoAbsEncoder::Send_/CanSend() failed\n");
    }
  }

 public:
  using PseudoAbsEncoder::PseudoAbsEncoder;

  void AttachSend(DistributedCAN& can, int dev_id) {
    attached_can_.enabled = true;
    attached_can_.can = &can;
    attached_can_.dev_id = dev_id;
  }

  void Update() {
    if (angle_.Update(this->GetAngle())) {
      Send_();
    }

    if (abs_ready_detector_.Update(this->IsAbsReady())) {
      SendAbsReady_();
    }
  }
};

class App {
 public:
  struct Config {
    struct {
      int id;
      int freqency;

      PinName rx, tx;
    } can;
    PseudoAbsEncoder::Config encoder_sm0;
    PseudoAbsEncoder::Config encoder_sm1;
    PseudoAbsEncoder::Config encoder_sm2;
    PseudoAbsEncoder::Config encoder_dummy;
  };

 private:
  DistributedCAN can_;

  DistributedPseudoAbsEncoder encoder_sm0_;
  DistributedPseudoAbsEncoder encoder_sm1_;
  DistributedPseudoAbsEncoder encoder_sm2_;
  DistributedPseudoAbsEncoder encoder_dummy_;

  Thread monitor_thread_;
  Thread sender_thread_;

  void MonitorThread() {
    char buf[80];
    while (1) {
      snprintf(buf, 80,
               "encoders: %d(%4.1lf)[%d] %d(%4.1lf)[%d] %d(%4.1lf)[%d] "
               "%d(%4.1lf)[%d\n",
               encoder_sm0_.GetPulses(), encoder_sm0_.GetAngle(),
               encoder_sm0_.IsAbsReady(), encoder_sm1_.GetPulses(),
               encoder_sm1_.GetAngle(), encoder_sm1_.IsAbsReady(),
               encoder_sm2_.GetPulses(), encoder_sm2_.GetAngle(),
               encoder_sm2_.IsAbsReady(), encoder_dummy_.GetPulses(),
               encoder_dummy_.GetAngle(), encoder_dummy_.IsAbsReady());
      AtomicPrint(buf);
      wait_ns(500E6);
    }
  }
  void SenderThread() {
    while (1) {
      encoder_sm0_.Update();
      encoder_sm1_.Update();
      encoder_sm2_.Update();
      encoder_dummy_.Update();
      wait_ns(50E6);
    }
  }

 public:
  App(Config const& config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        encoder_sm0_(config.encoder_sm0),
        encoder_sm1_(config.encoder_sm1),
        encoder_sm2_(config.encoder_sm2),
        encoder_dummy_(config.encoder_dummy) {
    encoder_sm0_.AttachSend(can_, 0x00);
    encoder_sm1_.AttachSend(can_, 0x01);
    encoder_sm2_.AttachSend(can_, 0x02);
    encoder_dummy_.AttachSend(can_, 0x03);
  }

  void Init() { can_.Init(); }

  void Main() {
    monitor_thread_.start(callback(this, &App::MonitorThread));
    sender_thread_.start(callback(this, &App::SenderThread));
    while (1) {
      ThisThread::sleep_for(1s);
    }
  }
};

int main2(int argc, char const* argv[]) {
  PseudoAbsEncoder encoder(PB_14, PB_15, PB_13);
  while (1) {
    printf("%4d(%4.1lf)\n", encoder.GetPulses(), encoder.GetAngle());
  }
  return 0;
}

int main(int argc, char const* argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);
  App::Config config{.can =
                         {
                             .id = CAN_ID,
                             .freqency = (int)1E6,
                             .rx = PB_8,
                             .tx = PB_9,
                         },
                     .encoder_sm0 = {.a = PA_6, .b = PC_9, .index = PC_7},
                     .encoder_sm1 = {.a = PB_14, .b = PB_15, .index = PB_13},
                     .encoder_sm2 = {.a = PA_0, .b = PB_10, .index = PA_1},
                     .encoder_dummy = {.a = PB_2, .b = PC_8, .index = PA_3}};
  // PB_4: LED

  printf("Initializing class\n");
  App app(config);
  printf("Initializing\n");
  app.Init();
  printf("Main...\n");
  app.Main();
  printf("main() ended\n");

  return 0;
}