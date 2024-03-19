#include <mbed.h>

#include "../identify.h"
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

/* class Color {
 public:
  uint32_t r;
  uint32_t g;
  uint32_t b;

  Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}

  Color(uint32_t rgb)
      : r((rgb >> 16) & 0xFF), g((rgb >> 8) & 0xFF), b(rgb & 0xFF) {}

  Color operator+(Color const &other) {
    return Color(r + other.r, g + other.g, b + other.b);
  }

  Color operator-(Color const &other) {
    return Color(r - other.r, g - other.g, b - other.b);
  }

  Color operator*(float const &other) {
    return Color(r * other, g * other, b * other);
  }

  Color operator*(int const &other) {
    return Color(r * other, g * other, b * other);
  }

  Color operator/(int const &other) {
    return Color(r / other, g / other, b / other);
  }

  uint32_t ToRGB() {
    uint8_t r = this->r > 255 ? 255 : this->r;
    uint8_t g = this->g > 255 ? 255 : this->g;
    uint8_t b = this->b > 255 ? 255 : this->b;

    return (r << 16) | (g << 8) | b;
  }
}; */

int main_3() {
  NeoPixel led(PB_2, 20);

  int i = 0;
  int j = 0;

  Color buf[20] = {0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                   0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                   0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                   0x000000, 0x000000, 0x000000, 0x000000, 0x00ff00};

  while (1) {
    led.Clear();

    led.PutPixel((20 + i + 0) % 20, buf[0].ToRGB());
    led.PutPixel((20 + i + 1) % 20, buf[1].ToRGB());
    led.PutPixel((20 + i + 2) % 20, buf[2].ToRGB());
    led.PutPixel((20 + i + 3) % 20, buf[3].ToRGB());
    led.PutPixel((20 + i + 4) % 20, buf[4].ToRGB());
    led.PutPixel((20 + i + 5) % 20, buf[5].ToRGB());
    led.PutPixel((20 + i + 6) % 20, buf[6].ToRGB());
    led.PutPixel((20 + i + 7) % 20, buf[7].ToRGB());
    led.PutPixel((20 + i + 8) % 20, buf[8].ToRGB());
    led.PutPixel((20 + i + 9) % 20, buf[9].ToRGB());
    led.PutPixel((20 + i + 10) % 20, buf[10].ToRGB());
    led.PutPixel((20 + i + 11) % 20, buf[11].ToRGB());
    led.PutPixel((20 + i + 12) % 20, buf[12].ToRGB());
    led.PutPixel((20 + i + 13) % 20, buf[13].ToRGB());
    led.PutPixel((20 + i + 14) % 20, buf[14].ToRGB());
    led.PutPixel((20 + i + 15) % 20, buf[15].ToRGB());
    led.PutPixel((20 + i + 16) % 20, buf[16].ToRGB());
    led.PutPixel((20 + i + 17) % 20, buf[17].ToRGB());
    led.PutPixel((20 + i + 18) % 20, buf[18].ToRGB());
    led.PutPixel((20 + i + 19) % 20, buf[19].ToRGB());

    led.Write();

    auto tick = 20 - 1 * i / 40.0f;
    int tick_int = tick < 0 ? 0 : (int)tick;

    ThisThread::sleep_for(tick_int * 1ms);
    i++;

    for (int k = 0; k < 19; k++) {
      buf[k] = (buf[k] * 39 + buf[k + 1]) / 40;
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
                              .revolver_change_id = 2,
                              .elevation_pid_id = 4,
                              .rotation_pid_id = 5,
                              .shot_l_factor_id = 6,
                              .shot_r_factor_id = 7,
                              .revolver_pid_id = 6,
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
  printf("    - sizeof(App): %d\n", sizeof(App));

  // main_mi();
  // main_3();
  // main_can();
  main_pro();
  return 0;
}
