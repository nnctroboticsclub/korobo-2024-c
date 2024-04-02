#include <chrono>
#include <iostream>

#include "simulation.hpp"

using namespace std::chrono_literals;

class SPITransferable {
  virtual void Transfer(std::vector<uint8_t>& vec) = 0;
};

class SPIBus {
  std::unordered_map<int, std::shared_ptr<SPITransferable>> devices;

 public:
  void AddDevice(std::shared_ptr<SPITransferable> device) {
    devices.push_back(device);
  }
};

class SPI {
  int bits;
  int mode;

 public:
  SPI() {}

  void format(int bits, int mode) {}

  void frequency(int hz) {
    simulation::Message msg{"SPI", "frequency"};
    msg.args.push_back(std::to_string(hz));
  }

  int write(int value) {
    simulation::Message msg{"SPI", "write"};
    msg.args.push_back(std::to_string(value));
  }

  int write(const char* tx_buffer, int tx_length, char* rx_buffer,
            int rx_length) {
    simulation::Message msg{"SPI", "write"};
    msg.args.push_back(std::string(tx_buffer, tx_length));
    msg.args.push_back(std::to_string(tx_length));
    msg.args.push_back(std::string(rx_buffer, rx_length));
    msg.args.push_back(std::to_string(rx_length));
  }

  int write(const char* tx_buffer, int tx_length) {
    simulation::Message msg{"SPI", "write"};
    msg.args.push_back(std::string(tx_buffer, tx_length));
    msg.args.push_back(std::to_string(tx_length));
  }

  int read(char* rx_buffer, int rx_length, char tx_value) {
    simulation::Message msg{"SPI", "read"};
    msg.args.push_back(std::string(rx_buffer, rx_length));
    msg.args.push_back(std::to_string(rx_length));
    msg.args.push_back(std::to_string(tx_value));
  }

  int read(char* rx_buffer, int rx_length) {
    simulation::Message msg{"SPI", "read"};
    msg.args.push_back(std::string(rx_buffer, rx_length));
    msg.args.push_back(std::to_string(rx_length));
  }
};

class DeviceContext {};

class Host {
  simulation::Server server;

 public:
  Host() {
    server.OnConnect([](std::shared_ptr<simulation::Driver> driver) {
      driver->OnReceive("CCa", "init", [](const simulation::Message& msg) {
        // Args: (rx, tx, frequency)
      });
      driver->OnReceive("CCa", "write", [](const simulation::Message& msg) {

      });
      driver->OnReceive("CCa", "read", [](const simulation::Message& msg) {
        // Args: ()
      });
      driver->OnReceive("OI2", "init", [](const simulation::Message& msg) {
        // Args: (sda, scl)
      });
      driver->OnReceive("OPw", "init", [](const simulation::Message& msg) {
        // Args: ()
      });

      driver->OnReceive("OPw", "period", [](const simulation::Message& msg) {
        // Args: (time_us)
      });

      driver->OnReceive("OPw", "pulsewidth",
                        [](const simulation::Message& msg) {
                          // Args: (value)
                        });
      driver->OnReceive("ODi", "init", [](const simulation::Message& msg) {
        // Args: ()
      });
      driver->OnReceive("ODi", "write", [](const simulation::Message& msg) {
        // Args: (bool)
      });
      driver->OnReceive("OI2", "write", [](const simulation::Message& msg) {
        // Args: (addr, repeated, data...)
      });
      driver->OnReceive("OI2", "read", [](const simulation::Message& msg) {
        // Args: (addr, len, repeated)
      });
    });
  }
};

int main(int argc, char const* argv[]) {
  Host host;

  while (1) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}