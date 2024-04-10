// #include "app.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <vector>

#include <cmath>

class NeoPixel {
  static constexpr int kResetSize = 48 * 2;
  static std::vector<uint8_t> reset;
  const size_t kLEDs;
  std::vector<uint8_t> data;
  std::shared_ptr<ISPI> spi_;

  void WriteByte(size_t index, uint8_t byte) {
    int byte_index = kResetSize + 4 * index;

    data[byte_index + 0] =
        ((byte >> 7) & 1 ? 0xE : 0x8) << 4 | ((byte >> 6) & 1 ? 0xE : 0x8);
    data[byte_index + 1] =
        ((byte >> 5) & 1 ? 0xE : 0x8) << 4 | ((byte >> 4) & 1 ? 0xE : 0x8);
    data[byte_index + 2] =
        ((byte >> 3) & 1 ? 0xE : 0x8) << 4 | ((byte >> 2) & 1 ? 0xE : 0x8);
    data[byte_index + 3] =
        ((byte >> 1) & 1 ? 0xE : 0x8) << 4 | ((byte >> 0) & 1 ? 0xE : 0x8);
  }

 public:
  NeoPixel(std::shared_ptr<ISPI> spi, size_t kLEDs) : kLEDs(kLEDs), spi_(spi) {
    data.resize(kResetSize + 4 * 3 * kLEDs);
    for (size_t i = 0; i < data.size(); i++) {
      data[i] = 0;
    }
  }

  void PutPixel(size_t index, uint32_t rgb) {
    uint8_t g = (rgb >> 16) & 0xFF;
    uint8_t r = (rgb >> 8) & 0xFF;
    uint8_t b = rgb & 0xFF;

    WriteByte(index * 3 + 0, r);
    WriteByte(index * 3 + 1, g);
    WriteByte(index * 3 + 2, b);
  }

  void Clear() {
    for (size_t i = 0; i < kLEDs; i++) {
      PutPixel(i, 0);
    }
  }
  void Write() {
    std::vector<uint8_t> rx(data.size());
    spi_->Transfer(data, rx);
  }

  void Debug() {
    for (size_t i = 0; i < data.size(); i++) {
      printf("%02x ", data[i]);
      if (i % 12 == 11) putchar(' ');
      if (i % 24 == 23) {
        printf("\n");
      }
    }
    printf("\n");
  }
};

Color hsv2color(float h, float s, float v) {
  float c = v * s;
  float x = c * (1 - std::abs(fmod(h / 60, 2) - 1));
  float m = v - c;

  float r, g, b;
  if (h < 60) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 120) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 180) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 240) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 300) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  return Color((r + m) * 255, (g + m) * 255, (b + m) * 255);
}

extern "C" void app_main(void) {
  /* App app;
  app.Init();
  app.Main(); */

  idf::SPIMaster spi_master{idf::SPINum(1), idf::MOSI(27), idf::MISO(19),
                            idf::SCLK(5)};

  auto dev = spi_master.create_dev(idf::CS(18), idf::Frequency(3.2E6));

  auto spi = std::make_shared<ESP32SPI>(dev);
  NeoPixel neopixel(spi, 7);
  neopixel.Clear();

  int i = 0;
  while (1) {
    for (size_t j = 0; j < 7; j++) {
      float h = (5 * i + j * 10) % 360;
      float s = 1;
      float v = cos(M_PI / 2 * ((i / 5 - j) % 7) / 6.0) * 0.1;

      neopixel.PutPixel(j, hsv2color(h, s, v).ToRGB());
    }
    neopixel.Write();

    vTaskDelay(pdMS_TO_TICKS(10));
    i++;
  }
}
