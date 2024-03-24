#pragma once

#include <functional>
#include <vector>
#include <string>

#include <esp_log.h>
#include <lwip/sockets.h>

class RoboTCPClient {
  int sock = -1;
  std::function<void(std::vector<uint8_t> const&)> on_recv;

  std::vector<std::vector<uint8_t>> recv_buffer;

 public:
  RoboTCPClient() {}

  void Connect(std::string ip) {
    sockaddr_in addr{
        .sin_family = PF_INET,
        .sin_port = htons(8001),
        .sin_addr = {.s_addr = INADDR_ANY},
    };

    this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_TCP);

    /* auto err = connect(this->sock, (sockaddr*)&addr, sizeof(addr));
    if (err < 0) {
      ESP_LOGE("RoboTCP", "Failed to connect to %s\n", ip.c_str());
      return;
    } else {
      ESP_LOGI("RoboTCP", "Connected to %s\n", ip.c_str());
    } */

    // Set BROADCAST
    int flag = 1;
    auto err = setsockopt(this->sock, SOL_SOCKET, SO_BROADCAST, (char*)&flag,
                          sizeof(int));
    if (err < 0) {
      ESP_LOGW("RoboTCP", "Failed to set SO_BROADCAST");
    } else {
      ESP_LOGI("RoboTCP", "SO_BROADCAST is set");
    }

    err = setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, (char*)&flag,
                     sizeof(int));
    if (err < 0) {
      ESP_LOGW("RoboTCP", "Failed to set SO_REUSEADDR");
    } else {
      ESP_LOGI("RoboTCP", "SO_REUSEADDR is set");
    }

    err = bind(this->sock, (sockaddr*)&addr, sizeof(addr));
    if (err < 0) {
      ESP_LOGE("RoboTCP", "Failed to connect to %s\n", ip.c_str());
      return;
    } else {
      ESP_LOGI("RoboTCP", "Connected to %s\n", ip.c_str());
    }
  }

  void Send(std::vector<uint8_t> const& data) {
    sockaddr_in dest = {.sin_family = PF_INET,
                        .sin_port = htons(8001),
                        .sin_addr = {.s_addr = inet_addr("255.255.255.255")}};

    std::vector<uint8_t> payload;
    payload.push_back((data.size() >> 24) & 0xff);
    payload.push_back((data.size() >> 16) & 0xff);
    payload.push_back((data.size() >> 8) & 0xff);
    payload.push_back(data.size() & 0xff);
    payload.insert(payload.end(), data.begin(), data.end());

    auto ret = sendto(this->sock, payload.data(), payload.size(), 0,
                      (struct sockaddr*)&dest, sizeof(dest));
  }

  void Thread() {
    std::vector<uint8_t> buf{0};
    socklen_t remote_addr_len = 0;
    while (1) {
      if (sock == -1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }

      std::vector<uint8_t> chunk;
      chunk.resize(0x10);
      auto ret = recvfrom(             //
          this->sock,                  //
          chunk.data(), chunk.size(),  //
          0,                           //
          nullptr,                     //
          &remote_addr_len             //
      );
      if (ret < 0) {
        ESP_LOGE("RoboTCP", "recv failed\n");
        close(this->sock);
        this->sock = -1;
        break;
      }
      chunk.resize(ret);

      buf.insert(buf.end(), chunk.begin(), chunk.end());

      // ESP_LOG_BUFFER_HEXDUMP("TCP  <-", chunk.data(), chunk.size(),
      // ESP_LOG_INFO);
      // ESP_LOG_BUFFER_HEXDUMP("BUF  <-", buf.data(), buf.size(),
      // ESP_LOG_INFO);

      auto ptr = buf.data();
      auto read_bytes = 0;
      while (buf.size() - read_bytes > 4) {
        uint32_t length = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
        read_bytes += 4;
        ptr += 4;

        if (length == 0 || length >= 0x20) {
          ESP_LOGE("RoboTCP", "Detected Malformed Data!!!");
          ESP_LOGE("RoboTCP", "Ignoring %d bytes", buf.size());
          read_bytes = buf.size();
          break;
        }

        std::vector<uint8_t> payload(ptr, ptr + length);
        read_bytes += length;
        ptr += length;

        ESP_LOG_BUFFER_HEXDUMP("Robo <-", payload.data(), payload.size(),
                               ESP_LOG_INFO);

        if (this->on_recv) {
          this->on_recv(payload);
        }
      }
      buf.erase(buf.begin(), buf.begin() + read_bytes);
    }
  }

  bool ConnectionEstablished() { return this->sock != -1; }

  void OnRecv(std::function<void(std::vector<uint8_t> const&)> cb) {
    this->on_recv = cb;
  }
};