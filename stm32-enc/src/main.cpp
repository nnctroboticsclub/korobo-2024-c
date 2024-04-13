#include "mbed.h"
#include "rtos.h"

#include <vector>
#include <sstream>

#include <mbed-robotics/distributed_pseudo_encoder.hpp>

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

class App {
  using DistributedPseudoAbsEncoder =
      robotics::mbed::DistributedPseudoAbsEncoder;
  using PseudoAbsEncoder = robotics::mbed::PseudoAbsEncoder;

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

  void MonitorThread() {
    char buf[80];
    while (1) {
      snprintf(
          buf, 80,
          "encoders p(a)[abs]: "
          "%d(%4.1lf)[%s] %d(%4.1lf)[%s] %d(%4.1lf)[%s] "
          "%d(%4.1lf)[%s]\n",
          encoder_sm0_.GetPulses(), encoder_sm0_.GetAngle(),
          encoder_sm0_.IsAbsReady() ? "OK" : "--", encoder_sm1_.GetPulses(),
          encoder_sm1_.GetAngle(), encoder_sm1_.IsAbsReady() ? "OK" : "--",
          encoder_sm2_.GetPulses(), encoder_sm2_.GetAngle(),
          encoder_sm2_.IsAbsReady() ? "OK" : "--", encoder_dummy_.GetPulses(),
          encoder_dummy_.GetAngle(), encoder_dummy_.IsAbsReady() ? "OK" : "--");
      AtomicPrint(buf);
      wait_ns(500E6);
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
    while (1) {
      ThisThread::sleep_for(1s);
    }
  }
};

int main(int argc, char const* argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);
  App::Config config{
      .can =
          {
              .id = CAN_ID,
              .freqency = (int)1E6,
              .rx = PB_8,
              .tx = PB_9,
          },
      .encoder_sm0 = {.a = PB_14, .b = PB_15, .index = PB_13},  // ok
      .encoder_sm1 = {.a = PA_0, .b = PB_10, .index = PA_1},    // ok
      .encoder_sm2 = {.a = PB_2, .b = PC_8, .index = PB_3},
      .encoder_dummy = {.a = PA_5, .b = PC_9, .index = PB_7}};
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