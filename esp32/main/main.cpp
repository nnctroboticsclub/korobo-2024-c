#include <functional>

#include <stdio.h>

#include <libstm-ota.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <gpio_cxx.hpp>
#include <driver/twai.h>

#include <esp_log.h>

class CANDriver {
 public:
  using Callback =
      std::function<void(uint32_t id, std::vector<uint8_t> const& data)>;
  using CallbackTag = uint32_t;

 private:
  twai_handle_t twai_driver_;
  std::vector<std::pair<CallbackTag, Callback>> callbacks_;
  bool bus_locked = false;

  static void AlertLoop(void* args) {
    static const char* TAG = "AlertWatcher#CANDriver";

    auto& self = *static_cast<CANDriver*>(args);
    auto driver = self.twai_driver_;

    while (1) {
      uint32_t alerts;
      auto status =
          twai_read_alerts_v2(driver, &alerts, 1000 / portTICK_PERIOD_MS);
      if (status == ESP_ERR_TIMEOUT) {
        continue;
      }
      if (status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read TWAI alerts: %s",
                 esp_err_to_name(status));
        continue;
      }
      if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(TAG, "Bus recovered");
        self.bus_locked = false;
        twai_start_v2(driver);
      }
      if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "Recovering Bus...");
        twai_initiate_recovery_v2(driver);
        self.bus_locked = true;
      }

      if (alerts & TWAI_ALERT_RX_DATA) {
        twai_message_t msg;
        auto status = twai_receive_v2(driver, &msg, 500 / portTICK_PERIOD_MS);
        if (status == ESP_ERR_TIMEOUT) {
          continue;
        }

        if (status != ESP_OK) {
          ESP_LOGE(TAG, "Failed to receive TWAI message: %s",
                   esp_err_to_name(status));
          continue;
        }

        std::vector<uint8_t> data(msg.data_length_code);
        std::copy(msg.data, msg.data + msg.data_length_code, data.begin());

        for (auto& cb : self.callbacks_) {
          cb.second(msg.identifier, data);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

 public:
  CANDriver() : twai_driver_(nullptr), callbacks_(), bus_locked(false) {}

  void Init(gpio_num_t tx, gpio_num_t rx) {
    static const char* TAG = "Init#CANDriver";

    twai_general_config_t general_config =
        TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, twai_mode_t::TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    general_config.rx_queue_len = 100;
    general_config.tx_queue_len = 0;
    general_config.alerts_enabled = TWAI_ALERT_ALL;

    auto status = twai_driver_install_v2(&general_config, &timing_config,
                                         &filter_config, &twai_driver_);
    if (status != ESP_OK) {
      ESP_LOGE(TAG, "Failed to install TWAI driver: %s",
               esp_err_to_name(status));
      return;
    }
    ESP_LOGI(TAG, "install TWAI driver sucessful");

    status = twai_start_v2(twai_driver_);
    if (status != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(status));
      return;
    }
    ESP_LOGI(TAG, "start TWAI driver sucessful");

    xTaskCreate(CANDriver::AlertLoop, "AlertLoop#CAN", 4096, this, 1, NULL);
  }

  bool SendStd(uint32_t id, std::vector<uint8_t> const& data) {
    static const char* TAG = "Send#CANDriver";

    if (data.size() > 8) {
      ESP_LOGE(TAG, "Data size must be <= 8");
      return false;
    }

    twai_message_t msg = {
        .ss = 1,
        .identifier = id,
        .data_length_code = (uint8_t)data.size(),
        .data = {0},
    };
    std::copy(data.begin(), data.end(), msg.data);

    auto status = twai_transmit_v2(twai_driver_, &msg, pdMS_TO_TICKS(1000));
    if (status != ESP_OK) {
      ESP_LOGE(TAG, "Failed to transmit TWAI message: %s",
               esp_err_to_name(status));
      return false;
    }
    return true;
  }

  void AddRxCallback(CallbackTag tag, Callback cb) {
    callbacks_.emplace_back(tag, cb);
  }
};

class App {
  CANDriver can_;

  stm32::ota::InitConfig& GetInitConfig() {
    using stm32::ota::InitConfig;

    static InitConfig init_config = {
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
                             }},
        .active_network_profile_id = 3,
        .primary_stm32_id = 2};

    return init_config;
  }

  void HandleRoboCtrl(std::vector<uint8_t>& data) {
    const int chunks = data.size() / 8;
    for (int i = 0; i < chunks; i++) {
      std::vector<uint8_t> chunk(data.begin() + i * 8,
                                 data.begin() + (i + 1) * 8);
      can_.SendStd(0x200 + i, chunk);

      ESP_LOG_BUFFER_HEX("CAN", chunk.data(), chunk.size());
    }

    std::vector last_chunk(data.begin() + chunks * 8, data.end());
    if (!last_chunk.empty()) {
      can_.SendStd(0x200 + chunks, last_chunk);
      ESP_LOG_BUFFER_HEX("CAN", last_chunk.data(), last_chunk.size());
    }
  }

  static esp_err_t RoboCtrl(httpd_req_t* req) {
    static const char* TAG = "RoboCtrl";

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    auto& app = *static_cast<App*>(req->user_ctx);

    if (req->method == HTTP_GET) {
      return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
      // ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d",
      // ret);
      return ret;
    }

    if (ws_pkt.len) {
      auto buf = std::vector<uint8_t>(ws_pkt.len + 1, 0);

      ws_pkt.payload = buf.data();
      ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
      }

      ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
      app.HandleRoboCtrl(buf);
    }
    return ret;
  }

 public:
  App() : can_() {}

  void Main() {
    can_.Init(GPIO_NUM_15, GPIO_NUM_4);

    auto& init_config = this->GetInitConfig();
    stm32::ota::OTAServer ota_server(idf::GPIONum(22), init_config);

    ota_server.OnHTTPDStart([this](httpd_handle_t server) {
      httpd_uri_t uri = {
          .uri = "/robo-ctrl",
          .method = HTTP_GET,
          .handler = App::RoboCtrl,
          .user_ctx = this,
          .is_websocket = true,
      };
      httpd_register_uri_handler(server, &uri);
    });

    ESP_LOGI("Main/CAN", "Sending test message");
    can_.SendStd(0x200, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06});
    ESP_LOGI("Main/CAN", "Adding callback");
    can_.AddRxCallback(0x201,
                       [](uint32_t id, std::vector<uint8_t> const& data) {
                         ESP_LOGI("CAN", "Got message with id: %ld", id);
                         ESP_LOG_BUFFER_HEX("CAN", data.data(), data.size());
                       });
    ESP_LOGI("Main/CAN", "Done");

    while (1) vTaskDelay(1);
  }
};

extern "C" void app_main(void) {
  App app;
  app.Main();
}
