#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "cmsis_os.h"

#include "librm.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern void CommunicateTask(void const* argument);

#ifdef __cplusplus
}
#endif

using namespace rm;
using namespace rm::device;

// enum class POWER { CHASSIS, GIMBAL, AMMOBOOSTER };

class ChassisControler final : public CanDevice {
 public:
  explicit ChassisControler(hal::CanInterface &can);
  ChassisControler() = delete;
  ~ChassisControler() override = default;
  u16 heat_real_;
  u16 heat_limit_;
  u16 cooling_speed_;
  f32 cooling_msg_;
  u8 robot_id_;
  u8 gimbal_power_state_;
  u8 chassis_power_state_;
  u8 ammo_power_state_;
  void RxCallback(const hal::CanMsg *msg) override;
  void SendChassisCommand();

 private:
  u8 tx_buf_[8]{0};
};

class HeatControler final : public CanDevice {
 public:
  explicit HeatControler(hal::CanInterface &can);
  HeatControler() = delete;
  ~HeatControler() override = default;

  void RxCallback(const hal::CanMsg *msg) override;
};

#endif /* COMMUNICATE_TASK_H */