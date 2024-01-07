#pragma once

namespace controller {

using RawPacketData = std::vector<uint8_t>;

struct RawPacket {
  uint8_t element_id;
  RawPacketData data;

  RawPacket(RawPacketData const& raw_data) : element_id(raw_data[0]), data{} {
    data.reserve(raw_data.size() - 1);
    for (int i = 1; i < raw_data.size(); i++) {
      data.push_back(raw_data[i]);
    }
  }

  uint8_t operator[](int index) const { return data[index]; }
};

}  // namespace controller