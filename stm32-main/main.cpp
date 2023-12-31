#include "mbed.h"
#include "rtos.h"
#include "BNO055.h"

#include "identify.h"

#include <vector>

using namespace std::chrono_literals;

using namespace rtos;

DigitalOut led1(LED1);

class KoroboCANDriver {
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

        // printf("E rd=%d td=%d\n", can_.rderror(), can_.tderror());

        ThisThread::sleep_for(10ms);
      }

      if (can_.read(msg)) {
        HandleMessage(msg);
      }
    }
  }

 public:
  KoroboCANDriver(int can_id, PinName rx, PinName tx, int freqency = 50E3)
      : can_id(can_id), can_(rx, tx, freqency), freqency_(freqency) {}

  void Init() { thread_.start(callback(this, &KoroboCANDriver::ThreadMain)); }

  // 1 -> success
  int Send(uint32_t id, std::vector<uint8_t> const &data) {
    return WriteMessage(id, data);
  }
};
/*
class Korobo2023C {
  void ThreadMain() {
    DigitalIn btn(USER_BUTTON);
    printf("I: Program started.\r\n");
    BNO055 sensor(PC_9, PA_8);

    if (!sensor.check()) {
      printf("E: Failed to connect BNO055.\r\n");
      return 0;
    }

    printf("I: Perform a power-on reset on BNO055\r\n");
    sensor.reset();

    printf("I: setpowermode()...\r\n");
    sensor.setpowermode(POWER_MODE_NORMAL);

    printf("I: setmode()...\r\n");
    sensor.setmode(OPERATION_MODE_COMPASS);

    printf("I: Setting units...\r\n");
    sensor.set_accel_units(DEGREES);
    sensor.set_angle_units(DEGREES);
    sensor.set_anglerate_units(DEGREES);

    while (true) {
      if (!sensor.check()) {
        printf("E: Failed to connect BNO055.\r\n");
        continue;
      }

      if (btn == 0) {
        sensor.get_angles();

        printf("D: setmode(ACCGYRO)\r\n");
        sensor.setmode(OPERATION_MODE_ACCGYRO);

        sensor.get_accel();
        sensor.get_gyro();

        printf("I: angle\r\n");
        printf("I:   yaw = %+10.5f\r\n", sensor.euler.yaw);
        printf("I:   pitch = %+10.5f\r\n", sensor.euler.pitch);
        printf("I:   roll = %+10.5f\r\n", sensor.euler.roll);
        printf("I:  acc x = %+10.5f, y = %+10.5f, z = %+10.5f\r\n",
               sensor.accel.x, sensor.accel.y, sensor.accel.z);
        printf("I: gyro x = %+10.5f, y = %+10.5f, z = %+10.5f\r\n",
               sensor.gyro.x, sensor.gyro.y, sensor.gyro.z);
        while (btn == 0)
          ;
      }
    }
  }
}; */

int main(int argc, char const *argv[]) {
  printf("main() started\n");
  KoroboCANDriver can(CAN_ID, PB_8, PB_9, 1E6);

  can.Init();

  while (1) {
    led1 = !led1;
    ThisThread::sleep_for(500ms);
  }

  return 0;
}