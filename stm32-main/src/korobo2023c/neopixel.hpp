#pragma once

#include <vector>

#include <mbed.h>

class NeoPixel {
  static constexpr int kResetSize = 48 * 2;
  static std::vector<uint8_t> reset;
  mbed::SPI spi;
  const size_t kLEDs;
  std::vector<uint8_t> data;

  void WriteByte(size_t index, uint8_t byte) {
    int byte_index = kResetSize + 4 * index;
    for (size_t bit_index = 0; bit_index < 4; bit_index += 1) {
      char pattern;

      pattern = (byte & 1) ? 0xE : 0x8;
      pattern <<= 4;
      byte >>= 1;

      pattern |= (byte & 1) ? 0xE : 0x8;
      byte >>= 1;

      data[byte_index] = pattern;

      byte_index++;
    }
  }

 public:
  NeoPixel(PinName pin, size_t kLEDs) : spi(pin, NC, NC), kLEDs(kLEDs) {
    spi.frequency(6.4E6);

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

  void Write() { spi.write((char *)data.data(), data.size(), nullptr, 0); }

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