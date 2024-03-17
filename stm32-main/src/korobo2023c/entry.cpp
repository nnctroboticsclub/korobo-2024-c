#include <mbed.h>
#include "main.hpp"

#include <WS2812.h>
#include <PixelArray.h>

#define WS2812_BUF 5

int main() {
  WS2812 ws{PB_5, WS2812_BUF};
  PixelArray px{WS2812_BUF};

  DigitalOut led{LED1};

  ws.useII(1);  // global
  ws.setII(1);  // I = 1 (u8)

  px.Set(0, 0x00002f);
  px.Set(1, 0x002f2f);
  px.Set(2, 0x002f00);
  px.Set(3, 0x2f2f00);
  px.Set(4, 0x2f0000);

  while (1) {
    ws.write(px.getBuf());

    led = !led;
    ThisThread::sleep_for(500ms);
  }
}