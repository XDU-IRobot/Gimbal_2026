#ifndef __RC_REFEREE_DATA_HPP__
#define __RC_REFEREE_DATA_HPP__

#include <queue>
#include <array>

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;

class RcReferee {
 public:
  RcReferee() = delete;
  explicit RcReferee(SerialInterface &serial);

  void Begin();
  void RxCallback(const std::vector<u8> &data, u16 rx_len);

 private:
  SerialInterface *serial_;
};

#endif /* __RC_REFEREE_DATA_HPP__ */