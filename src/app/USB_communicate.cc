#include "USB_communicate.hpp"
#include "communicate_task.hpp"
#include "attitude_task.hpp"

AimbotFrame_SCM_t Aimbot_s;
GimabalImuFrame_SCM_t GimabalImu;
extern ChassisCommunicator *chassis_communicator;
extern INS_t INS;

/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
void UsbReceive(uint8_t *rx_data, uint8_t len) {
  if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    switch (rx_data[1]) {
      case 0x02:
        memcpy(&Aimbot_s, rx_data, len);
        Aimbot_s.YawRelativeAngle = Aimbot_s.Yaw;
        Aimbot_s.PitchRelativeAngle = Aimbot_s.Pitch;
        break;
        
      default:
        break;
    }
  }
}

/**
 * @brief          Usb数据发送
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @param[in]      数据id
 * @retval         none
 */
void UsbSendMessage(uint8_t *address, uint16_t len, uint8_t id) {
  address[0] = 0x55;
  address[1] = id;
  address[len - 1] = 0xff;
  CDC_Transmit_FS(address, len);
}

void GimbalImuSend() {
  GimabalImu.TimeStamp = 0;
  GimabalImu.q0 = INS.q[0];
  GimabalImu.q1 = INS.q[1];
  GimabalImu.q2 = INS.q[2];
  GimabalImu.q3 = INS.q[3];
  GimabalImu.robot_id = (chassis_communicator->robot_id() == 1) ? 103 : 3;
  UsbSendMessage((uint8_t *)&GimabalImu, (uint16_t)sizeof(GimabalImu), 0x1);
}