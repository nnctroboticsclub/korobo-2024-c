#pragma once

#include <string>
#include <vector>

#include <lwip/sockets.h>

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