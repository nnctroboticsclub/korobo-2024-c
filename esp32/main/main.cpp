#include "app.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <vector>

#include <cmath>

#include <robotics/utils/neopixel.hpp>

#include <spi_host_cxx.hpp>

using Color = robotics::utils::Color;

extern "C" void app_main(void) {
  App app;
  app.Init();
  app.Main();

  /* idf::SPIMaster spi_master{idf::SPINum(1), idf::MOSI(27), idf::MISO(19),
                            idf::SCLK(5)};

  auto dev = spi_master.create_dev(idf::CS(18), idf::Frequency(3.2E6));

  auto spi = std::make_shared<robotics::datalink::SPI>(dev);
  robotics::utils::NeoPixel neopixel(spi, 7);
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
  } */
}
