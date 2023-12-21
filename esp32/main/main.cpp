#include <stdio.h>

#include <libstm-ota.hpp>
#include <gpio_cxx.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" void app_main(void) {
  using stm32::ota::InitConfig;
  InitConfig init_config = {
      .uarts = {InitConfig::Uart{
          .port = 1,
          .baud_rate = 9600,
          .tx = GPIO_NUM_17,
          .rx = GPIO_NUM_16,
          .parity = UART_PARITY_DISABLE,
      }},
      .spi_buses = {InitConfig::SPIBus{
          .port = 2,
          .miso = GPIO_NUM_19,
          .mosi = GPIO_NUM_23,
          .sclk = GPIO_NUM_18,
      }},
      .stm32bls = {InitConfig::STM32BL{
          .id = 1,
          .spi_port_id = 2,
          .cs = GPIO_NUM_5,
      }},
      .stm32s = {InitConfig::STM32{
          .id = 2,
          .reset = GPIO_NUM_22,
          .boot0 = GPIO_NUM_21,
          .bl_id = 1,
      }},
      .serial_proxies = {InitConfig::SerialProxy{.id = 1, .uart_port_id = 1}},
      .network_profiles = {InitConfig::NetworkProfile{
          .id = 2,
          .is_ap = true,
          .is_static = false,
          .ssid = "ESP32",
          .password = "esp32-network",
          .hostname = "esp32",
          .ip = 0xc0a80001,
          .subnet = 0xffffff00,
          .gateway = 0xc0a80001,
      }},
      .active_network_profile_id = 2,
      .primary_stm32_id = 2};

  stm32::ota::OTAServer ota_server(idf::GPIONum(22), init_config);

  printf("Entering busy loop\n");

  while (1) vTaskDelay(1);
}
