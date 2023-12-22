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
  using Callback = std::function<void(CANDriver& driver, uint32_t id,
                                      std::vector<uint8_t> const& data)>;
  using CallbackTag = uint32_t;

 private:
  twai_handle_t twai_driver_;
  std::vector<std::pair<CallbackTag, Callback>> callbacks_;
  bool bus_locked = false;

  static void AlertLoop(void* args) {
    static const char* TAG = "AlertWatcher#CANDriver";

    auto self = static_cast<CANDriver*>(args);
    auto driver = self->twai_driver_;

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

      if (alerts & TWAI_ALERT_ERR_ACTIVE) {
        ESP_LOGE(TAG, "TWAI_ALERT_ERR_ACTIVE");
      }
      if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(TAG, "Bus recovered");
        self->bus_locked = false;
        twai_start_v2(driver);
      }
      if (alerts & TWAI_ALERT_ARB_LOST) {
        ESP_LOGW(TAG, "Twai arb lost");
      }
      if (alerts & TWAI_ALERT_TX_FAILED) {
        ESP_LOGW(TAG, "Twai tx failed");
      }
      if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
        ESP_LOGW(TAG, "Twai rx queue full");
      }
      if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "Recovering Bus...");
        twai_initiate_recovery_v2(driver);
        self->bus_locked = true;
      }
      if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
        ESP_LOGE(TAG, "Twai rx fifo overrun!!!");
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

        std::vector<uint8_t> data(msg.data, msg.data + msg.data_length_code);
        for (auto& cb : self->callbacks_) {
          cb.second(*self, msg.identifier, data);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

 public:
  CANDriver(gpio_num_t tx, gpio_num_t rx) {
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

    xTaskCreate(CANDriver::AlertLoop, "AlertLoop#CAN", 2048, this, 1, NULL);
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

    return init_config;
  }

  void Main() {
    if (0)
      xTaskCreate(
          [](void* args) {
            auto app = static_cast<App*>(args);
            auto& init_config = app->GetInitConfig();
            stm32::ota::OTAServer ota_server(idf::GPIONum(22), init_config);
          },
          "OTA Server", 4096, this, 1, nullptr);
  }
};

extern "C" void app_main(void) {
  CANDriver can(GPIO_NUM_15, GPIO_NUM_4);
  can.AddRxCallback(0x00000000, [](CANDriver&, uint32_t id,
                                   std::vector<uint8_t> const& data) {
    printf("Received CAN message on id=%lx: ", id);
    for (auto& byte : data) {
      printf("%02x ", byte);
    }
    printf("\n");
  });

  unsigned char i = 0;

  while (1) {
    auto status = can.SendStd(0x123, std::vector<uint8_t>{i});
    if (!status) {
      printf("Failed to send CAN message\n");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    i++;
  }
}
