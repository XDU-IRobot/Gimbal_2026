#include "communicate_task.hpp"
#include "gimbal_task.hpp"

#include <iostream>

using namespace rm;
using namespace rm::device;
using namespace rm::modules::algorithm::utils;

ChassisCommunicator *chassis_communicator = nullptr;
ChassisCommunicator::ChassisCommunicator(hal::CanInterface &can) : CanDevice(can, 0x100) {}
extern Gimbal *gimbal;
extern hal::Can can1;

void ChassisCommunicator::RxCallback(const hal::CanMsg *msg)
{
  GimbalRequestStatePacket_.heat_real = ((u16)msg->data[0] << 8 | (u16)msg->data[1]);
  GimbalRequestStatePacket_.cooling_speed = ((u16)msg->data[2] << 8 | (u16)msg->data[3]);
  GimbalRequestStatePacket_.heat_limit = ((u16)msg->data[4] << 8 | (u16)msg->data[5]);
  GimbalRequestStatePacket_.cooling_msg = ((f32)msg->data[6]);
  GimbalRequestStatePacket_.robot_id = (0x01 & (u8)msg->data[7]);
  GimbalRequestStatePacket_.gimbal_power_state = ((0x10 & (u8)msg->data[7]) >> 4);
  GimbalRequestStatePacket_.chassis_power_state = ((0x20 & (u8)msg->data[7]) >> 5);
  GimbalRequestStatePacket_.ammo_power_state = ((0x40 & (u8)msg->data[7]) >> 6);
}

void ChassisCommunicator::SendChassisCommand()
{
  tx_buf_[0] = gimbal->ChassisMoveXRequest();
  tx_buf_[1] = gimbal->ChassisMoveYRequest();
  tx_buf_[2] = gimbal->ChassisStateRequest();
  tx_buf_[3] = gimbal->UiChange();
  tx_buf_[4] = gimbal->GetTargetFlag();
  tx_buf_[5] = gimbal->SuggestFireFlag();
  tx_buf_[6] = gimbal->AimSpeedChange();
  this->can_->Write(0x120, tx_buf_, 8);
}

extern "C"
{
  void CommunicateTask(void const *argument)
  {
    chassis_communicator = new ChassisCommunicator{can1};
    for (;;)
    { // 1000Hz
      chassis_communicator->SendChassisCommand();
      osDelay(1);
    }
  }
}
