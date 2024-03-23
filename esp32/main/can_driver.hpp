#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <queue>

#include <driver/twai.h>

class CANDriver {
 public:
  using Callback =
      std::function<void(uint32_t id, std::vector<uint8_t> const& data)>;
  using MessageID = uint32_t;

 private:
  twai_handle_t twai_driver_;
  std::unordered_map<MessageID, std::vector<Callback>> callbacks_;
  std::vector<Callback> rx_callbacks_;
  std::vector<Callback> tx_callbacks_;

  std::vector<std::pair<uint16_t, std::vector<uint8_t>>> tx_buffer_;

  int bits_per_sample_ = 0;
  std::queue<int> bits_per_samples_ = std::queue<int>();
  float bus_load_ = 0.0f;
  bool bus_locked = false;

  void DropTxBuffer() { tx_buffer_.clear(); }

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

      if (alerts & TWAI_ALERT_TX_FAILED) {
        ESP_LOGE(TAG, "TX failed");
        self.bus_locked = true;
        self.DropTxBuffer();
      }

      if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        ESP_LOGI(TAG, "Bus recovered");
        twai_start_v2(driver);
      }
      if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "Recovering Bus...");
        twai_initiate_recovery_v2(driver);
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

        if (self.bus_locked) {
          self.bus_locked = false;
          ESP_LOGI(TAG, "Bus established!");
          ESP_LOGI(TAG, "  - Message: 0x%08lx", msg.identifier);
          ESP_LOGI(TAG, "  - DLC: %d", msg.data_length_code);
          ESP_LOGI(TAG, "  - Data: %02x %02x %02x %02x %02x %02x %02x %02x",
                   msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                   msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        }

        std::vector<uint8_t> data(msg.data_length_code);
        std::copy(msg.data, msg.data + msg.data_length_code, data.begin());

        for (auto& cb : self.callbacks_[msg.identifier]) {
          cb(msg.identifier, data);
        }

        for (auto& cb : self.rx_callbacks_) {
          cb(msg.identifier, data);
        }
      }
    }
  }

  static void TxBufferSender(void* args) {
    auto self = static_cast<CANDriver*>(args);

    while (1) {
      if (!self->bus_locked && self->tx_buffer_.size() > 0) {
        auto buffer = self->tx_buffer_;
        self->tx_buffer_.clear();

        for (auto& [id, data] : buffer) {
          self->DoSendStd(id, data);
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
  }

  static void BitSampleThread(void* args) {
    auto self = static_cast<CANDriver*>(args);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(100));
      self->bits_per_samples_.push(self->bits_per_sample_);
      self->bits_per_sample_ = 0;

      if (self->bits_per_samples_.size() > 10) {
        self->bits_per_samples_.pop();
      }

      int sum = 0;
      for (int i = 0; i < self->bits_per_samples_.size(); i++) {
        sum += self->bits_per_samples_.front();
      }
      self->bus_load_ = sum / 1000.0f / (1000 * 1000 * 1000);
    }
  }

  void AddBitSample(int bits) { bits_per_sample_ += bits; }

  bool DoSendStd(uint32_t id, std::vector<uint8_t> const& data) {
    static const char* TAG = "Send#CANDriver";

    if (data.size() > 8) {
      ESP_LOGE(TAG, "Data size must be <= 8");
      return false;
    }

    if (bus_locked) {
      ESP_LOGW(TAG, "Droped cached data, id=%#5lx, DLC=%d\n", id, data.size());
      return false;
    }

    twai_message_t msg = {
        .extd = 0,
        .identifier = id,
        .data_length_code = (uint8_t)data.size(),
        .data = {0},
    };
    std::copy(data.begin(), data.end(), msg.data);

    for (auto& cb : tx_callbacks_) {
      cb(id, data);
    }

    auto status = twai_transmit_v2(twai_driver_, &msg, pdMS_TO_TICKS(1000));
    if (status != ESP_OK) {
      ESP_LOGE(TAG, "Failed to transmit TWAI message: %s",
               esp_err_to_name(status));
      return false;
    }
    return true;
  }

 public:
  CANDriver() : twai_driver_(nullptr), callbacks_(), bus_locked(false) {}

  void Init(gpio_num_t tx, gpio_num_t rx) {
    static const char* TAG = "Init#CANDriver";
    twai_general_config_t general_config =
        TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, twai_mode_t::TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    general_config.rx_queue_len = 50;
    general_config.tx_queue_len = 50;
    general_config.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED |
                                    TWAI_ALERT_BUS_RECOVERED |
                                    TWAI_ALERT_BUS_OFF;

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

    OnRx([this](uint32_t id, std::vector<uint8_t> const& data) {
      AddBitSample(1 + 11 + 1 + 1 + 1 + 4 + 8 * data.size() + 15 + 1 + 1 + 1 +
                   7);
    });
    OnTx([this](uint32_t id, std::vector<uint8_t> const& data) {
      AddBitSample(1 + 11 + 1 + 1 + 1 + 4 + 8 * data.size() + 15 + 1 + 1 + 1 +
                   7);
    });

    xTaskCreate(CANDriver::AlertLoop, "AlertLoop#CAN", 4096, this, 1, NULL);
    xTaskCreate(CANDriver::BitSampleThread, "BitSampleThread#CAN", 4096, this,
                1, NULL);

    xTaskCreate(CANDriver::TxBufferSender, "TxBufferSender#CAN", 4096, this, 1,
                NULL);
  }

  void SendStd(uint32_t id, std::vector<uint8_t> const& data) {
    if (bus_locked) {
      return;
    }

    if (tx_buffer_.size() > 50) {
      ESP_LOGW("CANDriver", "Tx buffer is full, dropping message");
      return;
    }
    this->tx_buffer_.emplace_back(id, data);
  }

  void OnMessage(uint32_t id, Callback cb) {
    if (callbacks_.find(id) == callbacks_.end()) {
      callbacks_.emplace(id, std::vector<Callback>());
    }
    callbacks_[id].emplace_back(cb);
  }

  void OnRx(Callback cb) { rx_callbacks_.emplace_back(cb); }
  void OnTx(Callback cb) { tx_callbacks_.emplace_back(cb); }

  float GetBusLoad() { return bus_load_; }
};