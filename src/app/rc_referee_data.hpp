#ifndef RC_TC_DATA_H
#define RC_TC_DATA_H

#include <queue>
#include <array>

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;

typedef struct {
  std::array<u8, 128> data;
  u16 len;
} referee_rx_data_t;

class RcReferee {
 public:
  RcReferee() = delete;
  explicit RcReferee(SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  SerialInterface *serial_;
};

#endif /* RC_TC_DATA_H */