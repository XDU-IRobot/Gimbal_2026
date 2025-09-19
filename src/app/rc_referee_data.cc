#include "rc_referee_data.hpp"

extern VT03 rc_remote;

RcReferee::RcReferee(SerialInterface &serial) : serial_(&serial) {
  static SerialRxCallbackFunction rx_callback =
      std::bind(&RcReferee::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
  this->serial_->AttachRxCallback(rx_callback);
}

void RcReferee::Begin() { this->serial_->Begin(); }

void RcReferee::RxCallback(const std::vector<u8> &data, u16 rx_len) {
  for (u16 i = 0; i < rx_len; i++) {
    rc_remote << data.at(i);
  }
}