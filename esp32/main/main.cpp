#include <functional>
#include <unordered_map>
#include <vector>
#include <queue>

#include <stdio.h>

#include <libstm-ota.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <gpio_cxx.hpp>
#include <driver/twai.h>

#include <esp_log.h>

#include <logic_analyzer_ws.h>
#include <esp_websocket_client.h>

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

  int bits_per_sample_ = 0;
  std::queue<int> bits_per_samples_ = std::queue<int>();
  float bus_load_ = 0.0f;
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

      if (alerts & TWAI_ALERT_TX_FAILED) {
        ESP_LOGE(TAG, "TX failed");
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

        for (auto& cb : self.callbacks_[msg.identifier]) {
          cb(msg.identifier, data);
        }

        for (auto& cb : self.rx_callbacks_) {
          cb(msg.identifier, data);
        }
      }
    }
  }

  static void MessageWatcher(void* args) {
    static const char* TAG = "MessageWatcher#CANDriver";

    auto& self = *static_cast<CANDriver*>(args);
    auto driver = self.twai_driver_;

    while (1) {
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

      for (auto& cb : self.callbacks_[msg.identifier]) {
        cb(msg.identifier, data);
      }

      for (auto& cb : self.rx_callbacks_) {
        cb(msg.identifier, data);
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
    general_config.alerts_enabled =
        TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_BUS_OFF;

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
    xTaskCreate(CANDriver::MessageWatcher, "MessageWatcher#CAN", 4096, this, 15,
                NULL);
    xTaskCreate(CANDriver::BitSampleThread, "BitSampleThread#CAN", 4096, this,
                1, NULL);
  }

  bool SendStd(uint32_t id, std::vector<uint8_t> const& data) {
    static const char* TAG = "Send#CANDriver";

    if (data.size() > 8) {
      ESP_LOGE(TAG, "Data size must be <= 8");
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

constexpr const uint8_t kDeviceId = 2;

class KoroboCANDriver {
 public:
  using PongListener = std::function<void(uint8_t device)>;

 private:
  CANDriver can_;
  std::vector<PongListener> pong_listeners_;

 public:
  KoroboCANDriver() : can_() {}

  void Init(gpio_num_t tx, gpio_num_t rx) {
    can_.Init(tx, rx);

    can_.OnMessage(0x80, [this](uint32_t id, std::vector<uint8_t> const& data) {
      can_.SendStd(0x81 + kDeviceId, {kDeviceId});
    });

    for (uint8_t device = 0; device < 15; device++) {
      can_.OnMessage(
          0x81 + device,
          [this, device](uint32_t id, std::vector<uint8_t> const& data) {
            for (auto& cb : pong_listeners_) {
              cb(device);
            }
          });
    }
  }

  void SendStd(uint32_t id, std::vector<uint8_t> const& data) {
    can_.SendStd(id, data);
  }

  void OnMessage(uint32_t id, CANDriver::Callback cb) {
    can_.OnMessage(id, cb);
  }

  void OnRx(CANDriver::Callback cb) { can_.OnRx(cb); }
  void OnTx(CANDriver::Callback cb) { can_.OnTx(cb); }

  float GetBusLoad() { return can_.GetBusLoad(); }

  void SendControl(std::vector<uint8_t> const& data) { SendStd(0x40, data); }

  void Ping() { SendStd(0x80, {}); }

  void OnPong(PongListener cb) { pong_listeners_.emplace_back(cb); }
};

class AppInitializer {
  int sock = -1;

  bool TestConnectivility(std::string ip) {
    ESP_LOGI("AppInit",
             "Testing TCP connectivility test: ip = %s (port = 8001)\n",
             ip.c_str());

    int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in addr{
        .sin_family = PF_INET,
        .sin_port = htons(8001),
        .sin_addr = {.s_addr = inet_addr(ip.c_str())},
    };

    auto err = connect(sock, (sockaddr*)&addr, sizeof(addr));
    close(sock);
    if (err < 0) {
      ESP_LOGE("AppInit", "%s is not available IP", ip.c_str());
      return false;
    } else {
      ESP_LOGI("AppInit", "%s is available ip", ip.c_str());
      return true;
    }
  }

  int BindSocket() {
    if (this->sock != -1) CloseSocket();

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (this->sock < 0) {
      ESP_LOGE("AppInit", "Failed to create socket");
      return -1;
    }

    sockaddr_in addr{
        .sin_family = PF_INET,
        .sin_port = htons(38000),
        .sin_addr = {.s_addr = htonl(INADDR_ANY)},
    };
    return bind(this->sock, (sockaddr*)&addr, sizeof(addr));
  }

  void CloseSocket() {
    if (this->sock != -1) {
      close(this->sock);
      this->sock = -1;
    }
  }

 public:
  AppInitializer() {}

  std::vector<std::string> RecvPacket() {
    char buf[1024];

    ESP_LOGI("AppInit", "Waiting for packet");
    auto bytes = recvfrom(this->sock, buf, sizeof(buf), 0, NULL, NULL);
    if (bytes < 0) {
      ESP_LOGE("AppInit", "recvfrom failed (when receiving 'ip_length')");
      return {};
    }

    ESP_LOGI("AppInit", "Packet Received");
    char* ptr = buf;

    uint32_t ip_count = ptr[0] << 24 | ptr[1] << 16 | ptr[2] << 8 | ptr[3];
    ptr += 4;

    std::vector<std::string> ips;
    for (uint32_t i = 0; i < ip_count; i++) {
      uint32_t ip_length = ptr[0] << 24 | ptr[1] << 16 | ptr[2] << 8 | ptr[3];
      ptr += 4;

      std::string ip;
      ip.resize(ip_length);
      std::copy(ptr, ptr + ip_length, ip.begin());
      ptr += ip_length;

      ESP_LOGI("AppInit", "- %s", ip.c_str());
      ips.push_back(ip);
    }

    return ips;
  }

  std::string Test1() {
    this->BindSocket();
    auto ips = this->RecvPacket();
    this->CloseSocket();

    for (auto ip : ips) {
      if (TestConnectivility(ip)) {
        ESP_LOGI("AppInit", "Test successful, using %s as the server",
                 ip.c_str());
        return ip;
      }
    }

    return "";
  }
};

class RoboTCPClient {
  int sock = -1;
  std::function<void(std::vector<uint8_t> const&)> on_recv;

 public:
  RoboTCPClient() {}

  void Connect(std::string ip) {
    sockaddr_in addr{
        .sin_family = PF_INET,
        .sin_port = htons(8001),
        .sin_addr = {.s_addr = inet_addr(ip.c_str())},
    };

    this->sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    auto err = connect(this->sock, (sockaddr*)&addr, sizeof(addr));
    if (err < 0) {
      ESP_LOGI("RoboTCP", "Failed to connect to %s\n", ip.c_str());
    } else {
      ESP_LOGI("RoboTCP", "Connected to %s\n", ip.c_str());
    }
  }

  void Send(std::vector<uint8_t> const& data) {
    std::vector<uint8_t> payload;
    payload.push_back((data.size() >> 24) & 0xff);
    payload.push_back((data.size() >> 16) & 0xff);
    payload.push_back((data.size() >> 8) & 0xff);
    payload.push_back(data.size() & 0xff);
    payload.insert(payload.end(), data.begin(), data.end());

    send(this->sock, payload.data(), payload.size(), 0);
  }

  void Thread() {
    while (1) {
      if (sock == -1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }

      std::vector<uint8_t> buf;

      buf.resize(4);
      auto bytes = recv(this->sock, buf.data(), 4, 0);
      if (bytes < 0) {
        printf("recv failed\n");
        close(this->sock);
        this->sock = -1;
        continue;
      }

      uint32_t length = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
      ESP_LOGI("RoboTCP", "Receiving %ld bytes", length);

      buf.erase(buf.begin(), buf.begin() + 4);
      buf.resize(length);

      bytes = recv(this->sock, buf.data(), length, 0);
      if (bytes < 0) {
        printf("recv failed\n");
        close(this->sock);
        this->sock = -1;
        continue;
      }

      if (on_recv) {
        on_recv(buf);
      }
    }
  }

  bool ConnectionEstablished() { return this->sock != -1; }

  void OnRecv(std::function<void(std::vector<uint8_t> const&)> cb) {
    this->on_recv = cb;
  }
};

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
            .tx = -1,
            .rx = 14,
            .parity = UART_PARITY_DISABLE,
        }},
        .spi_buses = {InitConfig::SPIBus{
            .port = 2,
            .miso = GPIO_NUM_33,
            .mosi = GPIO_NUM_25,
            .sclk = GPIO_NUM_26,
        }},
        .stm32bls = {InitConfig::STM32BL{
            .id = 1,
            .spi_port_id = 2,
            .cs = GPIO_NUM_27,
        }},
        .stm32s = {InitConfig::STM32{
            .id = 2,
            .reset = GPIO_NUM_16,
            .boot0 = GPIO_NUM_17,
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
             }},
        .active_network_profile_id = 3,
        .primary_stm32_id = 2};

    return init_config;
  }
  stm32::ota::OTAServer StartOTAServer() {
    auto& init_config = this->GetInitConfig();
    stm32::ota::OTAServer ota_server(idf::GPIONum(22), init_config);

    ota_server.OnHTTPDStart([this](httpd_handle_t server) {
      logic_analyzer_register_uri_handlers(server);
    });

    return ota_server;
  }

  void SendCANtoRoboWs(uint16_t id, std::vector<uint8_t> const& data) {
    static std::vector<char> buffer;

    std::vector<uint8_t> payload = data;
    payload.insert(payload.begin(), id);

    buffer.insert(buffer.end(), payload.begin(), payload.end());

    if (client.ConnectionEstablished()) {
      /* ESP_LOG_BUFFER_HEXDUMP("WS -->", payload.data(), payload.size(),
                             ESP_LOG_INFO); */
      client.Send(payload);
    }
  }

  void StartTCPClient() {
    ESP_LOGI("RoboTCP", "Connecting to %s", server_ip.c_str());
    client.Connect(server_ip);
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
        "PingTask", 4096, this, 1, NULL);

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
        "LoadReporting", 4096, this, 1, NULL);

    xTaskCreate(
        [](void* args) {
          auto& app = *static_cast<App*>(args);
          app.client.Thread();
        },
        "TCPClient", 4096, this, 1, NULL);

    stm32::ota::OTAServer ota_server = this->StartOTAServer();

    AppInitializer app_initializer;
    this->server_ip = app_initializer.Test1();
    client.Connect(server_ip);

    while (1) {
      if (!client.ConnectionEstablished()) {
        AppInitializer app_initializer;
        this->server_ip = app_initializer.Test1();
        client.Connect(server_ip);
      }

      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
};

extern "C" void app_main(void) {
  App app;
  app.Init();
  app.Main();
}
