#include "mbed.h"

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <cstdio>

#include <functional>

#include "rtos.h"
#include "../simulation.hpp"
#include "PinName.h"

namespace mbed {
namespace pseudo {

HAL_StatusTypeDef hal::HAL_CAN_ConfigFilter(
    FDCAN_HandleTypeDef* hcan, CAN_FilterConfTypeDef* sFilterConfig) {
  return HAL_OK;
}

CANMessage::CANMessage(uint32_t id, const uint8_t* data, uint8_t len,
                       hal::CANFormat format, bool type)
    : id(id), len(len), format(format), type(type) {
  if (data) {
    std::memcpy(this->data, data, len);
  }
}

CAN::CAN(PinName rx, PinName tx, int frequency) {}
void CAN::lock() {}
void CAN::unlock() {}

int CAN::write(CANMessage msg) {
  throw simulation::NotImplemenetedError("CAN Write");
  return 1;
}
int CAN::read(CANMessage& msg, int handle) {
  throw simulation::NotImplemenetedError("CAN Read");
  return 0;
}
void CAN::reset() {}

void CAN::attach(Callback<void()> func, IrqType type) {}

int CAN::rderror() { return 0; }
int CAN::tderror() { return 0; }

I2C::I2C(PinName sda, PinName scl) {}
void I2C::write(int address, const char* data, int length, bool repeated) {
  throw simulation::NotImplemenetedError("I2C Write");
}
void I2C::read(int address, char* data, int length, bool repeated) {
  throw simulation::NotImplemenetedError("I2C Read");
}

void I2C::frequency(int hz) {
  // throw simulation::NotImplemenetedError("I2C Frequency");
}

SPI::SPI(PinName mosi, PinName miso, PinName sclk) {}
void SPI::frequency(int hz) {
  // throw simulation::NotImplemenetedError("SPI Frequency");
}
void SPI::write(const char* data, int length, char* receive,
                int receive_length) {
  throw simulation::NotImplemenetedError("SPI Write");
}

void SPI::format(int bits, int mode) {
  throw simulation::NotImplemenetedError("SPI Format");
}

Timer::Timer() {}
void Timer::start() {}

PwmOut::PwmOut(PinName pin) {}
void PwmOut::period_us(int period) {
  throw simulation::NotImplemenetedError("PWM Period");
}
void PwmOut::pulsewidth_us(int pulsewidth) {
  throw simulation::NotImplemenetedError("PWM Pulsewidth");
}

DigitalOut::DigitalOut(PinName pin) {}
void DigitalOut::write(bool value) {
  this->value = value;
  throw simulation::NotImplemenetedError("GPIO Out");
}

DigitalOut::operator bool() { return false; }
bool DigitalOut::operator=(bool value) {
  write(value);
  return value;
}

}  // namespace pseudo
using namespace pseudo;
}  // namespace mbed

using namespace mbed;