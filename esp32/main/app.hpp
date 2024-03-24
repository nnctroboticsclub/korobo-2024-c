#pragma once

#include <string>

#include <libstm-ota.hpp>
#include <esp_log.h>
#include <logic_analyzer_ws.h>

#include "distributed_can.hpp"
#include "robo_tcp_client.hpp"

class App {
  KoroboCANDriver can_;
  std::string server_ip;
  RoboTCPClient client;

  stm32::ota::InitConfig& GetInitConfig() {
    using stm32::ota::InitConfig;

    static InitConfig init_config = {
        .uarts = {InitConfig::Uart{
            .port = 1,
            .baud_rate = 9600,
            .tx = 17,
            .rx = 16,
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
        .network_profiles =
            {//
             InitConfig::NetworkProfile{
                 .id = 2,
                 .is_ap = true,
                 .is_static = true,
                 .ssid = "ESP32",
                 .password = "esp32-network",
                 .hostname = "esp32",
                 .ip = 0xc0a80001,
                 .subnet = 0xffffff00,
                 .gateway = 0xc0a80001,
             },
             InitConfig::NetworkProfile{
                 .id = 3,
                 .is_ap = false,
                 .is_static = false,
                 .ssid = "3-303-Abe 2.4Ghz",
                 .password = "syochnetwork",
                 .hostname = "esp32",
                 .ip = 0,
                 .subnet = 0,
                 .gateway = 0,
             },
             InitConfig::NetworkProfile{
                 .id = 4,
                 .is_ap = false,
                 .is_static = false,
                 .ssid = "Robonnct-2G4",
                 .password = "robonnct_wlan",
                 .hostname = "esp32",
                 .ip = 0,
                 .subnet = 0,
                 .gateway = 0,
             },
             InitConfig::NetworkProfile{
                 .id = 10,
                 .is_ap = false,
                 .is_static = false,
                 .ssid = "Pixel_4846",
                 .password = "aaaabbbb",
                 .hostname = "esp32",
                 .ip = 0,
                 .subnet = 0,
                 .gateway = 0,
             },
             InitConfig::NetworkProfile{
                 .id = 11,
                 .is_ap = false,
                 .is_static = false,
                 .ssid = "KanatasiPhone",
                 .password = "6cyZ-Kb9T-stcf-PqDV",
                 .hostname = "esp32",
                 .ip = 0,
                 .subnet = 0,
                 .gateway = 0,
             },
             InitConfig::NetworkProfile{
                 .id = 12,
                 .is_ap = false,
                 .is_static = false,
                 .ssid = "me34011v_0414",
                 .password = "12345678",
                 .hostname = "esp32",
                 .ip = 0,
                 .subnet = 0,
                 .gateway = 0,
             }},
        .active_network_profile_id = 12,
        .primary_stm32_id = 2};

    return init_config;
  }
  stm32::ota::OTAServer StartOTAServer() {
    auto& init_config = this->GetInitConfig();
    stm32::ota::OTAServer ota_server(idf::GPIONum(34), init_config);

    ota_server.OnHTTPDStart([this](httpd_handle_t server) {
      logic_analyzer_register_uri_handlers(server);
    });

    return ota_server;
  }

  void SendCANtoRoboWs(uint16_t id, std::vector<uint8_t> const& data) {
    std::vector<uint8_t> payload = data;
    payload.insert(payload.begin(), id);

    if (client.ConnectionEstablished()) {
      /* ESP_LOG_BUFFER_HEXDUMP("WS -->", payload.data(), payload.size(),
                             ESP_LOG_INFO); */
      client.Send(payload);
    }
  }

 public:
  App() : can_() {}

  void Init() {
    ESP_LOGI("Manager", "Init");
    ESP_LOGI("Manager", "- CAN");
    can_.Init(GPIO_NUM_15, GPIO_NUM_4);

    can_.OnPong(
        [this](uint8_t device) { this->SendCANtoRoboWs(0xff, {device}); });

    can_.OnMessage(0xa0, [this](uint32_t id, std::vector<uint8_t> const& data) {
      if (data.size() < 1) {
        ESP_LOGW("Manager", "CAN message 0xa0 has no payload.");
        return;
      }
      uint32_t msg_id = data[0];
      std::vector<uint8_t> payload(data.begin() + 1, data.end());
      this->SendCANtoRoboWs(msg_id, payload);
    });

    client.OnRecv([this](std::vector<uint8_t> const& data) {
      /* ESP_LOG_BUFFER_HEXDUMP("WS <--", data.data(), data.size(),
       * ESP_LOG_INFO); */
      can_.SendControl(data);
    });
  }

  void Main() {
    xTaskCreate(
        [](void* args) {
          auto& app = *static_cast<App*>(args);
          while (1) {
            app.can_.Ping();
            vTaskDelay(pdMS_TO_TICKS(5000));
          }
        },
        "PingTask", 8192, this, 1, NULL);

    xTaskCreate(
        [](void* args) {
          auto& app = *static_cast<App*>(args);
          int i = 0;
          while (1) {
            auto load = app.can_.GetBusLoad();
            auto load_int = static_cast<uint32_t>(load * 0xffffffff);
            app.SendCANtoRoboWs(0xfe, {static_cast<uint8_t>(load_int >> 24),
                                       static_cast<uint8_t>(load_int >> 16),
                                       static_cast<uint8_t>(load_int >> 8),
                                       static_cast<uint8_t>(load_int)});
            i++;
            vTaskDelay(pdMS_TO_TICKS(500));
          }
        },
        "LoadReporting", 8192, this, 1, NULL);

    xTaskCreate(
        [](void* args) {
          auto& app = *static_cast<App*>(args);
          while (1) {
            app.can_.KeepAlive();
            vTaskDelay(pdMS_TO_TICKS(10));
          }
        },
        "KeepAlive", 8192, this, 1, NULL);

    xTaskCreate(
        [](void* args) {
          auto& app = *static_cast<App*>(args);
          app.client.Thread();
        },
        "TCPClient", 16384, this, 4, NULL);

    stm32::ota::OTAServer ota_server = this->StartOTAServer();

    // AppInitializer app_initializer;
    // this->server_ip = app_initializer.Test1();
    client.Connect("");

    while (1) {
      if (!client.ConnectionEstablished()) {
        // AppInitializer app_initializer;
        // this->server_ip = app_initializer.Test1();
        client.Connect("");
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
};