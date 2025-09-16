#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "cmsis_os.h"

#include "librm.hpp"

using namespace rm;
using namespace rm::device;

class ChassisCommunicator final : public CanDevice
{
public:
  explicit ChassisCommunicator(hal::CanInterface &can);
  ChassisCommunicator() = delete;
  ~ChassisCommunicator() override = default;

  u16 heat_real() { return GimbalRequestStatePacket_.heat_real; }
  u16 cooling_speed() { return GimbalRequestStatePacket_.cooling_speed; }
  u16 heat_limit() { return GimbalRequestStatePacket_.heat_limit; }
  f32 cooling_msg() { return GimbalRequestStatePacket_.cooling_msg; }
  u8 robot_id() { return GimbalRequestStatePacket_.robot_id; }
  u8 gimbal_power_state() { return GimbalRequestStatePacket_.gimbal_power_state; }
  u8 chassis_power_state() { return GimbalRequestStatePacket_.chassis_power_state; }
  u8 ammo_power_state() { return GimbalRequestStatePacket_.ammo_power_state; }

  void RxCallback(const hal::CanMsg *msg) override;
  void SendChassisCommand();

private:
  struct GimbalRequestState_t
  {
    u16 heat_real;     
    u16 heat_limit;
    u16 cooling_speed;
    f32 cooling_msg;
    u8 robot_id;
    u8 gimbal_power_state;
    u8 chassis_power_state;
    u8 ammo_power_state;
  };

  GimbalRequestState_t GimbalRequestStatePacket_; // 云台控制需求数据

  u8 tx_buf_[8]{0};
};

#ifdef __cplusplus
extern "C"
{
#endif

  extern void CommunicateTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_TASK_H */