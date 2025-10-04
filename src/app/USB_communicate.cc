#include "USB_communicate.hpp"
#include "communicate_task.hpp"
#include "attitude_task.hpp"

#include "usbd_cdc_if.h"

AimbotFrame_SCM_t aimbot_frame;
GimabalImuFrame_SCM_t gimbal_imu_frame;
extern ChassisCommunicator *chassis_communicator;
extern INS_t INS;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief          Usb接收数据
 * @param[in]      数据地址指针
 * @param[in]      数据长度
 * @retval         none
 */
void USBReceive(uint8_t *rx_data, uint8_t len) {
  if (rx_data[0] == 0x55 && rx_data[len - 1] == 0xFF) {
    switch (rx_data[1]) {
      case 0x02:
        memcpy(&aimbot_frame, rx_data, len);
        aimbot_frame.YawRelativeAngle = aimbot_frame.Yaw;
        aimbot_frame.PitchRelativeAngle = aimbot_frame.Pitch;
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
void USBSendMessage(uint8_t *address, uint16_t len, uint8_t id) {
  address[0] = 0x55;
  address[1] = id;
  address[len - 1] = 0xff;
  CDC_Transmit_FS(address, len);
}

/**
 * @brief          数据处理与发送
 * @retval         none
 */
void GimbalImuSend() {
  gimbal_imu_frame.TimeStamp = 0;
  gimbal_imu_frame.q0 = INS.q[0];
  gimbal_imu_frame.q1 = INS.q[1];
  gimbal_imu_frame.q2 = INS.q[2];
  gimbal_imu_frame.q3 = INS.q[3];
  gimbal_imu_frame.robot_id = (chassis_communicator->robot_id() == 1) ? 103 : 3;
  USBSendMessage((uint8_t *)&gimbal_imu_frame, (uint16_t)sizeof(gimbal_imu_frame), 0x01);
}

#ifdef __cplusplus
}
#endif
