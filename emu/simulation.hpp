#pragma once

#include <cstring>

#include <stdexcept>

#include <sys/un.h>
#include <sys/socket.h>

namespace simulation {

class NotImplemenetedError : public std::exception {
  char *message;

 public:
  NotImplemenetedError(const char *message) {
    this->message = new char[strlen(message) + 20];
    memset(this->message, 0, strlen(message) + 20);
    strcpy(this->message, "Not implemented: ");
    strcat(this->message, message);
  }
  ~NotImplemenetedError() { delete[] message; }
  const char *what() const noexcept override { return message; }
};

class Driver {
  int fd;
  Driver(std::string path) {}

 public:
  void Connect() {
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
      throw std::runtime_error("[Driver] Failed to create socket");
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, "emu/emu.sock");

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      throw std::runtime_error("[Driver] Failed to connect to socket");
    }
  }

 private:
  static Driver *instance;

 public:
  static Driver *GetInstance() {
    if (instance == nullptr) {
      instance = new Driver("emu/emu.sock");
    }
    return instance;
  }
};

}  // namespace simulation