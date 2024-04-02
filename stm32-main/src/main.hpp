#include <cinttypes>
#include <mbed.h>

#include "identify.h"
#include "app.hpp"

#include "neopixel.hpp"

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

int main_2() {
  NeoPixel led(PB_2, 20);

  int i = 0;

  while (1) {
    led.Clear();

    /* led.PutPixel((20 + i + 0) % 20, 0x320064);
    led.PutPixel((20 + i + 1) % 20, 0x960032);
    led.PutPixel((20 + i + 2) % 20, 0xFF0000);
    led.PutPixel((20 + i + 3) % 20, 0xff0000);
    led.PutPixel((20 + i + 4) % 20, 0x960032);
    led.PutPixel((20 + i + 5) % 20, 0x320064);
    led.PutPixel((20 + i + 6) % 20, 0x0000FF);
    led.PutPixel((20 + i + 7) % 20, 0x0000FF);
    led.PutPixel((20 + i + 8) % 20, 0x0000FF);
    led.PutPixel((20 + i + 9) % 20, 0x0000FF);
    led.PutPixel((20 + i + 10) % 20, 0x0000FF);
    led.PutPixel((20 + i + 11) % 20, 0x0000FF);
    led.PutPixel((20 + i + 12) % 20, 0x0000FF);
    led.PutPixel((20 + i + 13) % 20, 0x0000FF);
    led.PutPixel((20 + i + 14) % 20, 0x0000FF);
    led.PutPixel((20 + i + 15) % 20, 0x0000FF);
    led.PutPixel((20 + i + 16) % 20, 0x0000FF);
    led.PutPixel((20 + i + 17) % 20, 0x0000FF);
    led.PutPixel((20 + i + 18) % 20, 0x0000FF);
    led.PutPixel((20 + i + 19) % 20, 0x0000FF); */

    /* led.PutPixel((20 + i + 0) % 20, 0x00ff00);
    led.PutPixel((20 + i + 1) % 20, 0x00ff00);
    led.PutPixel((20 + i + 2) % 20, 0x00ff00);
    led.PutPixel((20 + i + 3) % 20, 0x00ff00);
    led.PutPixel((20 + i + 4) % 20, 0x00ff00);
    led.PutPixel((20 + i + 5) % 20, 0x000000);
    led.PutPixel((20 + i + 6) % 20, 0x000000);
    led.PutPixel((20 + i + 7) % 20, 0x000000);
    led.PutPixel((20 + i + 8) % 20, 0x000000);
    led.PutPixel((20 + i + 9) % 20, 0x000000);
    led.PutPixel((20 + i + 10) % 20, 0x000000);
    led.PutPixel((20 + i + 11) % 20, 0x000000);
    led.PutPixel((20 + i + 12) % 20, 0x000000);
    led.PutPixel((20 + i + 13) % 20, 0x000000);
    led.PutPixel((20 + i + 14) % 20, 0x000000);
    led.PutPixel((20 + i + 15) % 20, 0x000000);
    led.PutPixel((20 + i + 16) % 20, 0x000000);
    led.PutPixel((20 + i + 17) % 20, 0x000000);
    led.PutPixel((20 + i + 18) % 20, 0x000000); // 65ms
    led.PutPixel((20 + i + 19) % 20, 0x000000); */

    /* led.PutPixel((20 + i + 0) % 20, 0x003208);
    led.PutPixel((20 + i + 1) % 20, 0x006416);
    led.PutPixel((20 + i + 2) % 20, 0x00ff32);
    led.PutPixel((20 + i + 3) % 20, 0x006416);
    led.PutPixel((20 + i + 4) % 20, 0x003208);
    led.PutPixel((20 + i + 5) % 20, 0x000000);
    led.PutPixel((20 + i + 6) % 20, 0x000000);
    led.PutPixel((20 + i + 7) % 20, 0x000000);
    led.PutPixel((20 + i + 8) % 20, 0x000000);
    led.PutPixel((20 + i + 9) % 20, 0x000000);
    led.PutPixel((20 + i + 10) % 20, 0x003208);
    led.PutPixel((20 + i + 11) % 20, 0x006416);
    led.PutPixel((20 + i + 12) % 20, 0x00ff32);
    led.PutPixel((20 + i + 13) % 20, 0x006416);
    led.PutPixel((20 + i + 14) % 20, 0x003208);
    led.PutPixel((20 + i + 15) % 20, 0x000000);
    led.PutPixel((20 + i + 16) % 20, 0x000000);
    led.PutPixel((20 + i + 17) % 20, 0x000000);
    led.PutPixel((20 + i + 18) % 20, 0x000000); // 40ms
    led.PutPixel((20 + i + 19) % 20, 0x000000); */

    led.Write();

    ThisThread::sleep_for(40ms);
    i++;
  }

  return 0;
}

int main_3() {
  NeoPixel led(PB_2, 20);

  int i = 0;
  int j = 0;

  Color buf[] = {0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                 0x000000, 0x000000, 0x000000, 0x000000, 0x004444};

  while (1) {
    led.Clear();

    led.PutPixel((/* i */ +0) % 20, buf[0].ToRGB());
    led.PutPixel((/* i */ +1) % 20, buf[1].ToRGB());
    led.PutPixel((/* i */ +2) % 20, buf[2].ToRGB());
    led.PutPixel((/* i */ +3) % 20, buf[3].ToRGB());
    led.PutPixel((/* i */ +4) % 20, buf[4].ToRGB());
    led.PutPixel((/* i */ +5) % 20, buf[5].ToRGB());
    led.PutPixel((/* i */ +6) % 20, buf[6].ToRGB());
    led.PutPixel((/* i */ +7) % 20, buf[7].ToRGB());
    led.PutPixel((/* i */ +8) % 20, buf[8].ToRGB());
    led.PutPixel((/* i */ +9) % 20, buf[9].ToRGB());
    led.PutPixel((/* i */ +10) % 20, buf[10].ToRGB());
    led.PutPixel((/* i */ +11) % 20, buf[11].ToRGB());
    led.PutPixel((/* i */ +12) % 20, buf[12].ToRGB());
    led.PutPixel((/* i */ +13) % 20, buf[13].ToRGB());
    led.PutPixel((/* i */ +14) % 20, buf[14].ToRGB());
    led.PutPixel((/* i */ +15) % 20, buf[15].ToRGB());
    led.PutPixel((/* i */ +16) % 20, buf[16].ToRGB());
    led.PutPixel((/* i */ +17) % 20, buf[17].ToRGB());
    led.PutPixel((/* i */ +18) % 20, buf[18].ToRGB());
    led.PutPixel((/* i */ +19) % 20, buf[19].ToRGB());

    led.Write();

    // ThisThread::sleep_for(100ms);

    auto tick = 20 - 1 * i / 40.0f;
    int tick_int = tick <= 0 ? 1 : (int)tick;

    ThisThread::sleep_for(tick_int * 1ms);

    for (int k = 0; k < 19; k++) {
      buf[k] = buf[k] * 49;
      buf[k] = buf[k] + buf[k + 1];
      buf[k] = buf[k] / 50;
    }
    i++;
    if (i % 1000 == 0) {
      auto old_color = buf[19].ToRGB();
      if (0)
        buf[19] = ((old_color >> 14) & 0x00ffff) ^
                  ((old_color << 7) & 0xffff00) ^ 0x0000ff;
      buf[19] = ((old_color >> 8) & 0x00ffff) | ((old_color << 16) & 0xff0000);
      j++;

      /* printf("LED: ");
      for (int k = 0; k < 20; k++) {
        printf("%06x ", buf[k].ToRGB());
      }
      printf("\n"); */
    }
  }

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
                      .upper =
                          (korobo2023c::Upper::Controller::Config){
                              .shot_joystick_id = 2,
                              .do_shot_id = 2,
                              .do_load_id = 4,
                              .shot_speed_id = 0,
                              .load_speed_id = 5,
                              .max_elevation_id = 1,
                              .revolver_change_id = 5,
                              .elevation_pid_id = 4,
                              .rotation_pid_id = 5,
                              .shot_l_factor_id = 6,
                              .shot_r_factor_id = 7,
                              .revolver_pid_id = 6,
                              .rotation_factor_id = 8,
                              .elevation_factor_id = 9,
                          },
                      .soft_emc_id = 3,
                      .esc_factor_0_id = 2,
                      .esc_factor_1_id = 3,
                      .esc_factor_2_id = 4,
                      .steer_0_inverse_id = 3,
                      .steer_1_inverse_id = 4,
                      .steer_2_inverse_id = 5,

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
      .swerve_origin_setting = true,
      .encoder_debug = false,
      .can1_debug = false};

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
  printf("    - sizeof(App): %" PRIuPTR "\n", sizeof(App));

  // main_mi();
  // main_3();
  // main_can();
  main_pro();
  return 0;
}
