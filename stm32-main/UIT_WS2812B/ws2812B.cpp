//--------------------------------------------------------
//  SPI を使って WS2812B を点灯するためのクラス
//      サポートするボード： Nucleo-F401RE, Nucleo-F446RE
//
//  2016/11/21, Copyright (c) 2016 MIKAMI, Naoki
//--------------------------------------------------------

#include "ws2812B.hpp"
#include "PeripheralPins.h"  // for pinmap_peripheral()

namespace Mikami {
WS2812B::WS2812B(PinName pin, bool inv)
    : spi_(pin, NC, NC),
      mySpi_((SPI_TypeDef *)pinmap_peripheral(pin, PinMap_SPI_MOSI)) {
  spi_.format(8, 0);
  // クロックを 23 MHz 以下で最大の値に設定
  //      F401RE: 21.0 MHz
  //      F446RE: 22.5 MHz
  spi_.frequency(22500000);

  if (!inv)
    fp = &WS2812B::SendByteNorm;
  else
    fp = &WS2812B::SendByteInv;
}

void WS2812B::Write(uint32_t x) {
  static const uint32_t bit23 = 0x800000;
  for (int n = 0; n < 24; n++) {
    if ((x & bit23) == bit23)
      T1HL();
    else
      T0HL();
    x <<= 1;
  }
}

void WS2812B::Write(uint32_t x, int k) {
  for (int n = 0; n < k; n++) Write(x);
}

void WS2812B::Clear(int k) {
  for (int n = 0; n < k; n++) Write(0x000000);
  Reset();
}

void WS2812B::Send3Bytes(uint8_t x0, uint8_t x1, uint8_t x2) {
  SendByte(x0);
  SendByte(x1);
  SendByte(x2);
}

void WS2812B::SendByteNorm(uint8_t x) {
  while ((mySpi_->SR & SPI_SR_TXE) != SPI_SR_TXE) {
  }
  mySpi_->DR = x;
}

void WS2812B::SendByteInv(uint8_t x) {
  while ((mySpi_->SR & SPI_SR_TXE) != SPI_SR_TXE) {
  }
  mySpi_->DR = ~x;
}
}  // namespace Mikami
