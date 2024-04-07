#include "main.hpp"

/* #include <stdio.h>

#include <robotics/network/dcan.hpp> */

int main() {
  main_switch();
  /* SimpleCAN can(PB_8, PB_9, 1E6);

  can.Init();

  can.OnRx([](uint32_t id, std::vector<uint8_t> const &data) {
    printf("Received: %08X; ", id);
    for (auto &d : data) {
      printf("%02X ", d);
    }
    printf("\n");
  });

  can.OnTx([](uint32_t id, std::vector<uint8_t> const &data) {
    printf("Transmitted: %08X; ", id);
    for (auto &d : data) {
      printf("%02X ", d);
    }
    printf("\n");
  });

  can.Send(0x200, std::vector<uint8_t>{0x00, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00});
  can.Send(0x1FF, std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00});

  while (1) wait_us(1E6); */
  return 0;
}
