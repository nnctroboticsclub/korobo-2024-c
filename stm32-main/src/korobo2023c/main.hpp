#include <mbed.h>
#include <ws2812B.hpp>

#include "../identify.h"
#include "app.hpp"

int main_mi() {
  ikakoMDC mdc{1, -50, 50, 0.001, 0.0, 2.7, 0, 0.000015, 0.01};
  ikarashiCAN_mk2 can{PB_5, PB_6, 0};
  mdc.set_speed(80);

  ikakoMDC_sender send{&mdc, 1, &can, 1};
  while (1) {
    printf("send(): %d\n", send.send());
    ThisThread::sleep_for(500ms);
  }
}
int main_led2() {
  mbed::SPI serial(PB_5, NC, NC);
  serial.frequency(6.4E6);

  int buf[] = {
      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,
  };

  std::vector<char> reset;
  reset.resize(48);
  for (size_t i = 0; i < reset.size(); i++) {
    reset[i] = 0;
  }

  const size_t data_len = 8 * sizeof(buf);  // 1byte = 8bit => 2byte
  std::vector<char> data(data_len);
  for (size_t i = 0; i < data_len; i++) {
    data[i] = 0;
  }

  for (size_t i = 0; i < sizeof(buf); i++) {
    for (size_t j = 0; j < 8; j++) {
      data[i * 8 + j] =  //
          ((buf[i] >> (7 - j)) & 1) ? 0xF8 : 0xE0;
    }
  }

  std::vector<char> payload;
  payload.insert(payload.end(), reset.begin(), reset.end());
  payload.insert(payload.end(), data.begin(), data.end());

  for (size_t i = 0; i < payload.size(); i++) {
    printf("%02x ", payload[i]);
    if (i % 24 == 23) {
      printf("\n");
    }
  }
  printf("\n");

  printf("Waiting for 1ms\n");
  ThisThread::sleep_for(1ms);
  printf("Sending\n");
  serial.write(payload.data(), payload.size(), nullptr, 0);
  printf("Sent\n");

  return 0;
}
int main_can() {
  auto *can = new DistributedCAN(1, PB_8, PB_9, 1000000);
  printf("Init!\n");
  can->Init();
  printf("Setting up handlers\n");
  can->OnRx([](uint16_t id, std::vector<uint8_t> data) {
    printf("%3hX Received: ", id);
    for (auto byte : data) {
      printf("%02x ", byte);
    }
    printf("\n");
  });
  can->OnTx([](uint16_t id, std::vector<uint8_t> data) {
    printf("%3hX Sent: ", id);
    for (auto byte : data) {
      printf("%02x ", byte);
    }
    printf("\n");
  });

  printf("Sending\n");
  auto ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);

  while (1) {
    ThisThread::sleep_for(100s);
  }
}
int main_pro() {
  App::Config config{
      .com =
          {
              .can =
                  {
                      .id = CAN_ID,
                      .freqency = (int)1E6,
                      .rx = PB_8,
                      .tx = PB_9,
                  },
              .driving_can =
                  {
                      .rx = PB_5,
                      .tx = PB_6,
                  },
              .controller_ids =
                  {
                      .swerve =
                          (controller::swerve::SwerveController::Config){
                              .joystick_id = 0,
                              .rot_right_45_id = 0,
                              .rot_left_45_id = 1,
                              .rotation_pid_enabled_id = 1,
                              .motor_0_pid_id = 0,
                              .motor_1_pid_id = 1,
                              .motor_2_pid_id = 2,
                              .angle_pid_id = 3,
                          },
                      .shot_joystick_id = 2,
                      .do_shot_id = 2,
                      .shot_speed_id = 0,
                      .max_elevation_id = 1,
                      .esc_factor_0_id = 2,
                      .esc_factor_1_id = 3,
                      .esc_factor_2_id = 4,
                  },
              .value_store_ids =
                  {.swerve =
                       (controller::swerve::SwerveValueStore<float>::Config){
                           .motor_0_encoder_id = 0,
                           .motor_1_encoder_id = 1,
                           .motor_2_encoder_id = 2}},
              .i2c =
                  {
                      .sda = PC_9,
                      .scl = PA_8,
                  },
              .swerve_esc_pins =
                  {
                      // PA_9
                      // PB_10
                      .swerve_pin_m0 = PB_13,
                      .swerve_pin_m1 = PB_14,
                      .swerve_pin_m2 = PB_15,
                  },

          },

      .swerve_config = {.angle_offsets = {0, 120, 240}},
      .swerve_origin_setting = false,
      .encoder_debug = false,
  };

  printf("Ctor\n");
  App app(config);
  printf("Init\n");
  app.Init();
  printf("end-\n");

  while (1) {
    ThisThread::sleep_for(100s);
  }

  return 0;
}

int main_switch() {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  printf("Build information:\n");
  printf("  - Build date: %s\n", __DATE__);
  printf("  - Build time: %s\n", __TIME__);
  printf("  - Analytics:\n");
  printf("    - sizeof(App): %d\n", sizeof(App));

  // main_mi();
  // main_led2();
  // main_can();
  main_pro();
  return 0;
}
