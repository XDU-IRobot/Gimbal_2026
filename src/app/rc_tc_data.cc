// #include "TcData.h"

// extern VT03 tcremote;

// TcData::TcData(SerialInterface &serial) : serial_(&serial) {
//   static SerialRxCallbackFunction rx_callback =
//       std::bind(&TcData::RxCallback, this, std::placeholders::_1, std::placeholders::_2);
//   this->serial_->AttachRxCallback(rx_callback);
// }

// void TcData::Begin() { this->serial_->Begin(); }

// void TcData::RxCallback(const std::vector<u8> &data, u16 rx_len) {
//   for (u16 i = 0; i < rx_len; i++) {
//     tcremote << data.at(i);
//   }
//   offlinecounter = 0;
//   offline = false;
// }