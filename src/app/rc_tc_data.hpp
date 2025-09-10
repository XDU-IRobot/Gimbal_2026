// #ifndef REFREE_DATA_H
// #define REFREE_DATA_H

// #include <queue>
// #include <array>

// #include "librm.hpp"

// using namespace rm;
// using namespace rm::hal;
// using namespace rm::device;

// typedef struct {
//   std::array<u8, 128> data;
//   u16 len;
// } referee_rx_data_t;

// class TcData {
//  public:
//   TcData() = delete;
//   explicit TcData(SerialInterface &serial);

//   void Begin();
//   void RxCallback(const std::vector<u8> &data, u16 rx_len);

//   bool offline{false};
//   u16 offlinecounter{0};

//  private:
//   SerialInterface *serial_;
// };

// #endif /* REFREE_DATA_H */