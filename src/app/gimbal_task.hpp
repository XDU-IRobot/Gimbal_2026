#ifndef __GIMBAL_TASK_HPP__
#define __GIMBAL_TASK_HPP__

#include "cmsis_os.h"
#include "usart.h"
#include "can.h"

#include "librm.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;
using namespace rm::modules::algorithm;
using namespace rm::modules::algorithm::utils;

class Gimbal {
 public:
  Gimbal();
  ~Gimbal() = default;

  i8 ChassisMoveXRequest() { return ChassisRequestStatePacket_.ChassisMoveXRequest; }
  i8 ChassisMoveYRequest() { return ChassisRequestStatePacket_.ChassisMoveYRequest; }
  u8 ChassisStateRequest() { return ChassisRequestStatePacket_.ChassisStateRequest; }
  u8 UiChange() { return ChassisRequestStatePacket_.UiChange; }
  u8 GetTargetFlag() { return ChassisRequestStatePacket_.GetTargetFlag; }
  u8 SuggestFireFlag() { return ChassisRequestStatePacket_.SuggestFireFlag; }
  i8 AimSpeedChange() { return ChassisRequestStatePacket_.AimSpeedChange; }

 private:
  // 状态机
  enum StateMachineType {
    GM_NO_FORCE,     // 无力模式
    GM_INIT,         // 初始化模式，底盘断电
    GM_MATCH,        // 比赛模式
    GM_TEST,         // 调试模式
    GM_AIMBOT,       // 自瞄模式
    GM_NOGIMBAL,     // 云台断电下的模式
    GM_HIGH_SPEED,   // 高速模式
    GM_SINGLE_WHEEL  // 单轮模式
  };

  struct ChassisRequestState_t {
    i8 ChassisMoveXRequest;  // x轴运动控制
    i8 ChassisMoveYRequest;  // y轴运动控制
    u8 ChassisStateRequest;  // 底盘运动状态：无力，测试，随动，小陀螺正转，小陀螺反转
    u8 UiChange;             // 开启Ui
    u8 GetTargetFlag;        // 自瞄状态
    u8 SuggestFireFlag;      // 建议开火
    i8 AimSpeedChange;       // 转速等级
    i8 reserve[1];           // 保留位
  };

  DmMotorSettings<DmMotorControlMode::kMit> *pitch_motor_settings_;  // 云台pitch电机设置
  DmMotor<DmMotorControlMode::kMit> *pitch_motor_;                   // 云台pitch电机对象

  GM6020 yaw_motor_;  // 云台yaw电机对象

  M3508 ammo_left_;   // 左摩擦轮对象
  M3508 ammo_right_;  // 右摩擦轮对象

  M3508 rotor_motor_;  // 拨盘电机对象

  StateMachineType StateMachine_;  // 当前状态

  RcSwitchState left_flag_;   // 遥控器左侧挡位
  RcSwitchState right_flag_;  // 遥控器右侧挡位

  ChassisRequestState_t ChassisRequestStatePacket_;  // 底盘控制需求数据

  PID<PIDType::kPosition> yaw_pid_speed_;         // 云台yaw轴默认速度pid
  RingPID<PIDType::kPosition> yaw_pid_position_;  // 云台yaw轴默认位置pid

  PID<PIDType::kPosition> pitch_pid_speed_;     // 云台pitch轴默认速度pid
  PID<PIDType::kPosition> pitch_pid_position_;  // 云台pitch轴默认位置pid

  PID<PIDType::kPosition> aimbot_yaw_pid_speed_;         // 云台yaw轴自瞄速度pid
  RingPID<PIDType::kPosition> aimbot_yaw_pid_position_;  // 云台yaw轴自瞄位置pid

  PID<PIDType::kPosition> aimbot_pitch_pid_speed_;     // 云台pitch轴自瞄速度pid
  PID<PIDType::kPosition> aimbot_pitch_pid_position_;  // 云台pitch轴自瞄位置pid

  PID<PIDType::kPosition> buff_yaw_pid_speed_;         // 云台yaw轴打符速度pid
  RingPID<PIDType::kPosition> buff_yaw_pid_position_;  // 云台yaw轴打符位置pid

  PID<PIDType::kPosition> buff_pitch_pid_speed_;     // 云台pitch轴打符速度pid
  PID<PIDType::kPosition> buff_pitch_pid_position_;  // 云台pitch轴打符位置pid

  PID<PIDType::kPosition> ammo_pid_left_;   // 左摩擦轮速度pid
  PID<PIDType::kPosition> ammo_pid_right_;  // 右摩擦轮速度pid

  PID<PIDType::kPosition> rotor_pid_speed_;         // 拨盘速度pid
  RingPID<PIDType::kPosition> rotor_pid_position_;  // 拨盘位置pid

  f32 gimbal_yaw_rc_;    // 云台yaw轴遥控数据
  f32 gimbal_pitch_rc_;  // 云台pitch轴遥控数据

  f32 chassis_x_rc_;  // 底盘x轴遥控数据
  f32 chassis_y_rc_;  // 底盘y轴遥控数据

  i16 ammo_flag_rc_;  // 摩擦轮遥控数据位

  i8 aim_speed_change_;  // 摩擦轮转速改变值

  f32 ammo_left_speed_;   // 左摩擦轮速度
  f32 ammo_right_speed_;  // 右摩擦轮速度

  u16 heat_limit_;      // 热量上限值
  u16 heat_real_;       // 热量实时值
  i16 heat_last_;       // 上一次热量值
  i16 heat_delaytime_;  // 热量发送延迟时间

  i16 back_turn_time_;  // 拨盘反转时长
  i16 back_turn_flag_;  // 拨盘反转触发时长 100

  f32 rotor_position_;         // 拨盘位置
  f32 last_rotor_position_;    // 上一次拨盘位置
  f32 rotor_target_position_;  // 拨盘目标位置
  u32 rotor_circle_flag_;      // 拨盘过圈标志

  u32 shoot_flag_;       // 开火标志
  u32 shoot_flag_last_;  // 上一次开火标志

  bool speed_change_flag_;  // 速度模式改变标志

  bool single_wheel_flag_;   // 单轮模式标志
  bool single_wheel_state_;  // 单轮模式状态

  bool DMEnable_;  // 4310电机使能标志

  bool DFflag_;   // 大符标志
  bool XFflag_;   // 小符标志
  bool DFstate_;  // 大符状态
  bool XFstate_;  // 小符状态

  const f32 once_circle_;           // 单圈编码值 17000f
  const f32 kyaw_speed_;            // 云台yaw轴速度前馈系数 1.2f
  const f32 kyaw_current_;          // 云台yaw轴电流前馈系数 -0.5f
  const f32 kchassis_xy_rc_;        // 底盘xy轴遥控前馈系数 100.0f
  const f32 ammo_init_speed_;       // 摩擦轮初始速度 6300.0f
  const f32 kammo_speed_change_;    // 摩擦轮速度改变系数 20.0f
  const f32 rotor_position_dalte_;  // 拨盘位置误差值 2000.0f
  const f32 rotor_init_speed_[4];   // 拨盘初始速度速度 {低等级正转，高等级正转，单发，反转}{2000.0f, 2500.0f, 2000.0f,
                                    // -1000.0f}
  const f32 sensitivity_x_;         // 云台x轴灵敏度 0.3f
  const f32 sensitivity_y_;         // 云台y轴灵敏度 0.2f
  const f32 kmouse_sensitivity_x_;  // 鼠标x轴灵敏系数 10.0f
  const f32 kmouse_sensitivity_y_;  // 鼠标y轴灵敏系数 10.0f
  const f32 highest_pitch_angle_;   // 云台pitch轴最高 35.0f
  const f32 lowest_pitch_angle_;    // 云台pitch轴最低 30.0f

 public:
  void GimbalInit();          // 云台初始化
  void StateUpdate();         // 云台状态基更新
  void GimbalUpdate();        // 云台状态更新
  void ChassisStateUpdate();  // 底盘控制状态更新

 private:
  void GimbalEnableUpdate();    // 云台电机使能计算
  void GimbalDisableUpdate();   // 云台电机失能计算
  void RotorEnableUpdate();     // 拨盘使能计算
  void RotorDisableUpdate();    // 拨盘失能计算
  void AmmoEnableUpdate();      // 摩擦轮机构使能计算
  void AmmoDisableUpdate();     // 摩擦轮机构失能计算
  void GimbalAimbotUpdate();    // 装甲板自瞄云台计算,控制权交给NUC
  void GimbalMatchUpdate();     // 比赛模式云台计算
  void RotorAimbotUpdate();     // 自瞄模式拨盘模式更新
  void RotorMatchUpdate();      // 比赛模式拨盘模式更新
  void DaMiaoMotorEnable();     // 达妙电机使能
  void DaMiaoMotorDisable();    // 达妙电机失能
  void GimbalRCDataUpdate();    // 遥控数据更新
  void AimbotDateUpdate();      // 自瞄云台目标值更新
  void MovePIDUpdate();         // 运动pid数据更新
  void AimPIDUpdate();          // 自瞄pid数据更新
  void BuffPIDUpdate();         // 打符pid数据更新
  void RotorMotorDataUpdate();  // 拨盘电机数据更新
};

#ifdef __cplusplus
extern "C" {
#endif

extern void GimbalTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_HPP__ */