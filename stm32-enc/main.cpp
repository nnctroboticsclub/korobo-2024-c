#include "mbed.h"
#include "rtos.h"
#include "QEI.h"

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"

using namespace std::chrono_literals;

using namespace rtos;

template <typename T>
class UpdateDetector {
 private:
  T value_;
  bool updated_;

 public:
  UpdateDetector() : value_(0), updated_(false) {}
  UpdateDetector(T const& value) : value_(value), updated_(false) {}

  bool Update(T const& value) {
    if (value_ != value) {
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
  static Timer timer_;

  QEI* qei_;  // calling RPM or RPS may causes a crash (because of gettime())
  InterruptIn* index_;

  void IndexRise() { qei_->qei_reset(); }

 public:
  struct Config {
    PinName a, b, index;
  };

  PseudoAbsEncoder(PinName A, PinName B, PinName index) {
    printf("PseudoAbsEncoder(%d, %d, %d)\n", A, B, index);
    qei_ = new QEI(A, B, NC, 200, &PseudoAbsEncoder::timer_);

    index_ = new InterruptIn(index);
    index_->rise(callback(this, &PseudoAbsEncoder::IndexRise));
    printf("PseudoAbsEncoder(%d, %d, %d) done\n", A, B, index);
  }

  PseudoAbsEncoder(Config const& config)
      : PseudoAbsEncoder(config.a, config.b, config.index) {}

  double GetAngle() { return qei_->getAngle(); }
  int GetPulses() { return qei_->getPulses(); }
};

Timer PseudoAbsEncoder::timer_;

class DistributedPseudoAbsEncoder {
 private:
  PseudoAbsEncoder encoder_;
  UpdateDetector<double> angle_;
  struct {
    bool enabled;
    DistributedCAN* can;
    int dev_id;
  } attached_can_;

  void Send_() {
    if (!attached_can_.enabled) return;

    uint16_t value = (int16_t)(encoder_.GetAngle() / 360.0f * 0x7FFF);

    std::vector<uint8_t> payload;
    payload.push_back(0x60 | attached_can_.dev_id);
    payload.push_back(value >> 8);
    payload.push_back(value & 0xFF);

    attached_can_.can->Send(0x61, payload);
  }

 public:
  DistributedPseudoAbsEncoder(PseudoAbsEncoder const& encoder)
      : encoder_(encoder) {}

  double GetAngle() { return encoder_.GetAngle(); }

  void AttachSend(DistributedCAN& can, int dev_id) {
    attached_can_.enabled = true;
    attached_can_.can = &can;
    attached_can_.dev_id = dev_id;
  }

  void Update() {
    if (angle_.Update(encoder_.GetAngle())) {
      Send_();
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
    while (1) {
      printf("encoders: %lf %lf %lf %lf\n", encoder_sm0_.GetAngle(),
             encoder_sm1_.GetAngle(), encoder_sm2_.GetAngle(),
             encoder_dummy_.GetAngle());
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

int main(int argc, char const* argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);
  App::Config config{.can =
                         {
                             .id = CAN_ID,
                             .freqency = (int)1E6,
                             .rx = PB_8,
                             .tx = PB_9,
                         },
                     .encoder_sm0 = {.a = PA_1, .b = PB_2, .index = PB_3},
                     .encoder_sm1 = {.a = PC_4, .b = PC_5, .index = PC_6},
                     .encoder_sm2 = {.a = PA_7, .b = PC_8, .index = PA_9},
                     .encoder_dummy = {.a = PB_13, .b = PB_14, .index = PB_15}};

  printf("Initializing class\n");
  App app(config);
  printf("Initializing\n");
  app.Init();
  printf("Main...\n");
  app.Main();
  printf("main() ended\n");

  return 0;
}