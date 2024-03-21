#pragma once

#include <vector>

#include <mbed.h>

class Color {
 public:
  float r;
  float g;
  float b;

  Color(float r, float g, float b) : r(r), g(g), b(b) {}

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
    uint8_t r = (uint8_t)(this->r > 255 ? 255 : this->r);
    uint8_t g = (uint8_t)(this->g > 255 ? 255 : this->g);
    uint8_t b = (uint8_t)(this->b > 255 ? 255 : this->b);

    return (r << 16) | (g << 8) | b;
  }
};

class NeoPixel {
  static constexpr int kResetSize = 48 * 2;
  static std::vector<uint8_t> reset;
  mbed::SPI spi;
  const size_t kLEDs;
  std::vector<uint8_t> data;

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

  void Write() {
    // Debug();
    spi.write((char *)data.data(), data.size(), nullptr, 0);
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