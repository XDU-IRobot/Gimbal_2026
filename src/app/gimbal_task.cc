#include "gimbal_task.hpp"
#include "communicate_task.hpp"
#include "attitude_task.hpp"
#include "USB_communicate.hpp"
#include "rc_referee_data.hpp"

Gimbal *gimbal;
VT03 rc_remote;
Can can1(hcan1);
Can can2(hcan2);

static Serial *remote_uart;
static DR16 *remote;

static DmMotorSettings<DmMotorControlMode::kMit> *pitch_motorsettings;
static DmMotor<DmMotorControlMode::kMit> *pitch_motor;

extern INS_t INS;
extern AimbotFrame_SCM_t aimbot_frame;
extern GimabalImuFrame_SCM_t gimbal_imu_frame;
extern ChassisCommunicator *chassis_communicator;

Gimbal::Gimbal()
    : yaw_motor_(can1, 4),

      ammo_left_(can2, 1),
      ammo_right_(can2, 2),

      rotor_motor_(can1, 3),

      StateMachine_(GM_NO_FORCE),

      left_flag_(RcSwitchState::kDown),
      right_flag_(RcSwitchState::kDown),

      ChassisRequestStatePacket_{0, 0, 0, 0, 0, 0, 0, 0},

      yaw_pid_speed_(400.0f, 0.0f, 400.0f, 25000.0f, 0.0f),
      yaw_pid_position_(12.0f, 0.0f, 800.0f, 10000.0f, 0.0f, 360.0f),

      pitch_pid_speed_(0.1f, 0.0f, 3.2f, 5.0f, 0.0f),
      pitch_pid_position_(2.0f, 0.0006f, 16.0f, 10000.0f, 600.0f),

      aimbot_yaw_pid_speed_(600.0f, 0.0f, 800.0f, 25000.0f, 0.0f),
      aimbot_yaw_pid_position_(30.0f, 0.0f, 1600.0f, 10000.0f, 0.0f, 360.0f),

      aimbot_pitch_pid_speed_(0.1f, 0.0f, 3.0f, 5.0f, 0.0f),
      aimbot_pitch_pid_position_(2.0f, 0.0007f, 15.0f, 10000.0f, 600.0f),

      buff_yaw_pid_speed_(380.0f, 0.0f, 600.0f, 25000.0f, 0.0f),
      buff_yaw_pid_position_(15.0f, 0.0f, 1200.0f, 10000.0f, 0.0f, 360.0f),

      buff_pitch_pid_speed_(0.12f, 0.0f, 3.2f, 5.0f, 0.0f),
      buff_pitch_pid_position_(2.0f, 0.0008f, 15.0f, 10000.0f, 600.0f),

      ammo_pid_left_(6.0f, 0.0f, 3.0f, 10000.0f, 0.0f),
      ammo_pid_right_(6.0f, 0.0f, 3.0f, 10000.0f, 0.0f),

      rotor_pid_speed_(20.0f, 0.0f, 0.1f, 15000.0f, 0.0f),

      gimbal_yaw_rc_(0.0),
      gimbal_pitch_rc_(0.0),

      chassis_x_rc_(0.0),
      chassis_y_rc_(0.0),

      ammo_flag_rc_(0),

      aim_speed_change_(0),

      ammo_left_speed_(0.0),
      ammo_right_speed_(0.0),

      heat_limit_(80),
      heat_real_(0),
      heat_last_(0),
      heat_delaytime_(0),

      back_turn_time_(0),
      back_turn_flag_(100),

      rotor_position_(0.0),
      last_rotor_position_(0.0),
      rotor_target_position_(0.0),
      rotor_circle_flag_(0),

      shoot_flag_(0),
      shoot_flag_last_(0),

      speed_change_flag_(0),

      single_wheel_flag_(0),
      single_wheel_state_(0),

      DMEnable_(0),

      DFflag_(0),
      XFflag_(0),
      DFstate_(0),
      XFstate_(0),

      once_circle_(17000.0f),
      kyaw_speed_(1.2f),
      kyaw_current_(-0.5f),
      kchassis_xy_rc_(100.0f),
      ammo_init_speed_(6300.0f),
      kammo_speed_change_(20.0f),
      rotor_position_dalte_(20.0f),
      rotor_init_speed_{2000.0f, 2500.0f, 2000.0f, -1000.0f},
      sensitivity_x_(0.3f),
      sensitivity_y_(0.2f),
      kmouse_sensitivity_x_(10.0f),
      kmouse_sensitivity_y_(10.0f),
      highest_pitch_angle_(35.0f),
      lowest_pitch_angle_(30.0f) {}

/**
 * @brief  初始化can,yaw、pitch电机初位置
 * @note
 * @tparam
 */
void Gimbal::GimbalInit() {
  can1.SetFilter(0, 0);
  can2.SetFilter(0, 0);
  can1.Begin();
  can2.Begin();

  remote_uart = new hal::Serial(huart3, 18, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma);
  remote = new DR16(*remote_uart);
  remote->Begin();

  pitch_motorsettings = new DmMotorSettings<DmMotorControlMode::kMit>{
      0x03, 0x02, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)};
  pitch_motor = new DmMotor<DmMotorControlMode::kMit>(can2, *pitch_motorsettings, true);

  Serial referee_uart(huart6, 128, hal::stm32::UartMode::kDma, hal::stm32::UartMode::kDma);
  RcReferee rcdata(referee_uart);
  rcdata.Begin();

  gimbal_yaw_rc_ = INS.yaw;      // 云台yaw初始化
  gimbal_pitch_rc_ = INS.pitch;  // 云台pitch初始化
}

/**
 * @brief  更新控制状态，无力，测试，初始化，比赛
 * @tparam StateMachineType 控制状态类型
 */
void Gimbal::StateUpdate() {
  if (chassis_communicator->gimbal_power_state() == 0) {
    switch (StateMachine_) {
      case GM_NO_FORCE:
        break;
      case GM_INIT:
        break;
      default:
        StateMachine_ = GM_NOGIMBAL;
    }
  } else {
    if (StateMachine_ == GM_NOGIMBAL) {
      osDelay(1000);
    }
    left_flag_ = remote->switch_l();  // 遥控器标志位更新
    right_flag_ = remote->switch_r();
    switch (right_flag_) {
      case RcSwitchState::kUp:
        // 右拨杆打到最上侧挡位
        if (left_flag_ == RcSwitchState::kUp) {
          StateMachine_ = GM_AIMBOT;  // 左拨杆拨到上侧，自瞄模式
        } else if (left_flag_ == RcSwitchState::kMid) {
          StateMachine_ = GM_SINGLE_WHEEL;  // 左拨杆拨到中间，云台有力底盘无力
        } else {
          StateMachine_ = GM_MATCH;  // 左拨杆拨到下侧，进入比赛模式，此时全部系统都上电工作
        }
        break;

      case RcSwitchState::kMid:
        // 右拨杆打到中间挡位
        if (left_flag_ == RcSwitchState::kUp) {
          StateMachine_ = GM_HIGH_SPEED;  // 左拨杆拨到上侧，高速模式
        } else if (left_flag_ == RcSwitchState::kMid) {
          StateMachine_ = GM_INIT;  // 左拨杆拨到中间，云台有力底盘无力
        } else {
          StateMachine_ = GM_TEST;  // 左拨杆拨到下侧，进入测试模式，此时除发射系统外都工作
        }
        break;

      case RcSwitchState::kDown:
        // 右拨杆打到最下侧挡位，此时全部全部无力
        StateMachine_ = GM_NO_FORCE;
        break;

      default:
        StateMachine_ = GM_NO_FORCE;  // 如果遥控器离线，进入无力模式
        break;
    }
  }
}

/**
 * @brief  更新云台所有电机相关状态
 * @tparam StateMachineType 控制状态类型
 */
void Gimbal::GimbalUpdate() {
  switch (StateMachine_) {
    case GM_NO_FORCE:         // 无力模式下，所有电机失能
      GimbalDisableUpdate();  // 云台电机失能计算
      AmmoDisableUpdate();    // 摩擦轮机构失能计算
      RotorDisableUpdate();   // 拨盘失能计算
      break;

    case GM_INIT:            // 初始化模式下，发射系统与拨盘电机失能
      GimbalEnableUpdate();  // 云台电机使能计算
      AmmoDisableUpdate();   // 摩擦轮机构失能计算
      RotorDisableUpdate();  // 拨盘失能计算
      break;

    case GM_TEST:            // 测试模式下，发射系统与拨盘电机失能
      GimbalEnableUpdate();  // 云台电机使能计算
      AmmoDisableUpdate();   // 摩擦轮机构失能计算
      RotorDisableUpdate();  // 拨盘失能计算
      break;

    case GM_SINGLE_WHEEL:    // 单轮模式下，发射系统与拨盘电机失能
      GimbalEnableUpdate();  // 云台电机使能计算
      AmmoDisableUpdate();   // 摩擦轮机构失能计算
      RotorDisableUpdate();  // 拨盘失能计算
      break;

    case GM_MATCH:          // 比赛模式下，所有电机正常工作
      GimbalMatchUpdate();  // 云台电机使能计算
      AmmoEnableUpdate();   // 摩擦轮机构使能计算
      RotorEnableUpdate();  // 拨盘使能计算
      break;

    case GM_AIMBOT:          // 自瞄测试模式下，云台控制权交给NUC，底盘断电，发射系统正常工作
      GimbalAimbotUpdate();  // 云台电机自瞄计算
      AmmoEnableUpdate();    // 摩擦轮机构使能计算
      RotorEnableUpdate();   // 拨盘使能计算
      break;

    case GM_HIGH_SPEED:      // 高速模式下，云台正常运动
      GimbalEnableUpdate();  // 云台电机使能计算
      AmmoDisableUpdate();   // 摩擦轮机构失能计算
      RotorDisableUpdate();  // 拨盘失能计算
      break;

    case GM_NOGIMBAL:
      GimbalDisableUpdate();  // 云台电机失能计算
      AmmoDisableUpdate();    // 摩擦轮机构失能计算
      RotorEnableUpdate();    // 拨盘使能计算
      break;

    default:                  // 错误状态，所有电机失能
      GimbalDisableUpdate();  // 云台电机失能计算
      AmmoDisableUpdate();    // 摩擦轮机构失能计算
      RotorDisableUpdate();   // 拨盘失能计算
      break;
  }
}

/**
 * @brief  更新向底盘发送信息
 * @tparam ChassisRequestStatePacket_ 向底盘发送can包数据
 */
void Gimbal::ChassisStateUpdate() {
  chassis_x_rc_ =
      Constrain(remote->right_x() / 660.0f - (f32)remote->key(RcKey::kA) + (f32)remote->key(RcKey::kD) -
                    (f32)(rc_remote.data().keyboard_key >> 2 & 0x01) + (f32)(rc_remote.data().keyboard_key >> 3 & 0x01),
                -1.0f, 1.0f);
  chassis_y_rc_ =
      Constrain(remote->right_y() / 660.0f + (f32)remote->key(RcKey::kW) - (f32)remote->key(RcKey::kS) +
                    (f32)(rc_remote.data().keyboard_key >> 0 & 0x01) - (f32)(rc_remote.data().keyboard_key >> 1 & 0x01),
                -1.0f, 1.0f);
  ChassisRequestStatePacket_.UiChange =
      (remote->key(RcKey::kR) == 1 || (rc_remote.data().keyboard_key >> 8 & 0x01) == 1) ? 1 : 0;  // Ui是否开启

  if ((aimbot_frame.AimbotState >> 0) & 0x01) {
    ChassisRequestStatePacket_.GetTargetFlag = 1;
    if ((aimbot_frame.AimbotState >> 1) & 0x01) {
      ChassisRequestStatePacket_.SuggestFireFlag = 1;
    } else {
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
    }
  } else {
    ChassisRequestStatePacket_.GetTargetFlag = 0;
    ChassisRequestStatePacket_.SuggestFireFlag = 0;
  }

  if (remote->key(RcKey::kCtrl) == 1 || rc_remote.data().keyboard_key >> 5 & 0x01) {
    if (remote->key(RcKey::kV) == 1 || rc_remote.data().keyboard_key >> 14 & 0x01) {
      aim_speed_change_ = -1;
    } else if (aim_speed_change_ == -1) {
      ChassisRequestStatePacket_.AimSpeedChange--;
      if (ChassisRequestStatePacket_.AimSpeedChange < -10) ChassisRequestStatePacket_.AimSpeedChange = -10;
      aim_speed_change_ = 0;
    }
    if (remote->key(RcKey::kB) == 1 || rc_remote.data().keyboard_key >> 15 & 0x01) {
      aim_speed_change_ = 1;
    } else if (aim_speed_change_ == 1) {
      ChassisRequestStatePacket_.AimSpeedChange++;
      if (ChassisRequestStatePacket_.AimSpeedChange > 10) ChassisRequestStatePacket_.AimSpeedChange = 10;
      aim_speed_change_ = 0;
    }
  }

  switch ((StateMachine_)) {
    case GM_TEST:  // 测试模式下，底盘正常运行，有陀螺
      ChassisRequestStatePacket_.ChassisMoveXRequest = (i8)(chassis_x_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisMoveYRequest = (i8)(chassis_y_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 0);
      ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 3);  // 清除第 4 位
      if (remote->dial() == 660 || remote->key(RcKey::kShift) == 1 ||
          (rc_remote.data().keyboard_key >> 4 & 0x01) == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 1);   // 第 2 位 置 1，此时小陀螺正转开启
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);  // 清除第 3 位
      } else if (remote->dial() == -660) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 2);   // 第3位 置 1，此时小陀螺反转开启
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 1);  // 清除第 2 位
      } else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 1);  // 清除第 2 位
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);  // 清除第 3 位
      }
      if (remote->key(RcKey::kC) == 1 || (rc_remote.data().keyboard_key >> 13 & 0x01) == 1) {
        speed_change_flag_ = 1;
      } else if (speed_change_flag_ == 1) {
        speed_change_flag_ = 0;
        ChassisRequestStatePacket_.ChassisStateRequest ^= (u8)(1 << 4);  // 第 5 位置 1，高速模式
      }
      break;

    case GM_SINGLE_WHEEL:  // 单轮模式下，底盘无陀螺
      ChassisRequestStatePacket_.ChassisMoveXRequest = (i8)(chassis_x_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisMoveYRequest = (i8)(chassis_y_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 0);  // 第 1 位置 1，底盘有力
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 3);  // 第 4 位置 1，单轮上坡模式
      for (int i = 1; i < 7; i++) {
        if (i != 3) ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << i);  // 清除第 2、3、5、6、7 位
      }
      break;

    case GM_MATCH:  // 比赛模式下，底盘正常运行，遥控器拨盘不控制小陀螺
      ChassisRequestStatePacket_.ChassisMoveXRequest = (i8)(chassis_x_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisMoveYRequest = (i8)(chassis_y_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1);        // 最低位 置 1，此时底盘有力
      ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);  // 清除第 3 位
      if (remote->key(RcKey::kShift) == 1 || (rc_remote.data().keyboard_key >> 4 & 0x01) == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 1);
      }  // 第2位 置1，此时小陀螺正转开启
      else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 1);  // 清除第 2 位
      }
      if (remote->key(RcKey::kC) == 1 || (rc_remote.data().keyboard_key >> 13 & 0x01) == 1) {
        speed_change_flag_ = 1;
      } else if (speed_change_flag_ == 1) {
        speed_change_flag_ = 0;
        ChassisRequestStatePacket_.ChassisStateRequest ^= (u8)(1 << 4);  // 第 5 位置 1，高速模式
      }

      if (remote->key(RcKey::kZ) == 1 || (rc_remote.data().keyboard_key >> 11 & 0x01) == 1) {
        single_wheel_flag_ = 1;
      } else if (single_wheel_flag_ == 1) {
        single_wheel_flag_ = 0;
        single_wheel_state_ ^= 1;
      }
      if (single_wheel_state_ == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 3);  // 第 4 位置 1
      } else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 3);  // 清除第 4 位
      }

      if (DFstate_ == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 5);   // 第 6 位置 1
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 6);  // 清除第 7 位
      } else if (XFstate_ == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 6);   // 第 7 位置 1
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 5);  // 清除第 6 位
      } else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 5);  // 清除第 6 位
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 6);  // 清除第 7 位
      }

      break;

    case GM_NOGIMBAL:  // 云台无力下，底盘正常运行，遥控器拨盘不控制小陀螺
      ChassisRequestStatePacket_.ChassisMoveXRequest = (i8)(chassis_x_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisMoveYRequest = (i8)(chassis_y_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisStateRequest = 0;
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      if (remote->key(RcKey::kShift) == 1 || (rc_remote.data().keyboard_key >> 4 & 0x01) == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 1);
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);
      }  // 第 2 位 置 1，此时小陀螺正转开启
      else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 1);
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);
      }  // 清除第 2、3 位
      break;

    case GM_HIGH_SPEED:
      ChassisRequestStatePacket_.ChassisMoveXRequest = (i8)(chassis_x_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisMoveYRequest = (i8)(chassis_y_rc_ * kchassis_xy_rc_);
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1);
      if (remote->dial() == 660 || (i16)remote->key(RcKey::kShift) == 1 ||
          (i16)(rc_remote.data().keyboard_key >> 4 & 0x01) == 1) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 1);  // 第2位 置1，此时小陀螺正转开启
      } else if (remote->dial() == -660) {
        ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 2);  // 第2位 置1，此时小陀螺反转开启
      } else {
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 1);  // 清除第 2 位
        ChassisRequestStatePacket_.ChassisStateRequest &= ~(u8)(1 << 2);  // 清除第 3 位
      }
      ChassisRequestStatePacket_.ChassisStateRequest |= (u8)(1 << 4);  // 第 5 位置 1
      break;

      // 其他模式下，均向底盘发送无力信息
    case GM_INIT:
      ChassisRequestStatePacket_.ChassisMoveXRequest = 0;
      ChassisRequestStatePacket_.ChassisMoveYRequest = 0;
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      ChassisRequestStatePacket_.ChassisStateRequest = 0;
      break;

    case GM_AIMBOT:
      ChassisRequestStatePacket_.ChassisMoveXRequest = 0;
      ChassisRequestStatePacket_.ChassisMoveYRequest = 0;
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      ChassisRequestStatePacket_.ChassisStateRequest = 0;
      break;

    case GM_NO_FORCE:
      ChassisRequestStatePacket_.ChassisMoveXRequest = 0;
      ChassisRequestStatePacket_.ChassisMoveYRequest = 0;
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      ChassisRequestStatePacket_.ChassisStateRequest = 0;
      break;

    default:
      ChassisRequestStatePacket_.ChassisMoveXRequest = 0;
      ChassisRequestStatePacket_.ChassisMoveYRequest = 0;
      ChassisRequestStatePacket_.AimSpeedChange = 0;
      ChassisRequestStatePacket_.SuggestFireFlag = 0;
      ChassisRequestStatePacket_.ChassisStateRequest = 0;
      break;
  }
}

/**
 * @brief 云台有力下的pid解算
 * @tparam
 */
void Gimbal::GimbalEnableUpdate() {
  DaMiaoMotorEnable();
  GimbalRCDataUpdate();
  MovePIDUpdate();
  gimbal_imu_frame.mode = 0x00;
  yaw_motor_.SetCurrent(static_cast<i16>(yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
  pitch_motor->SetPosition(0, 0, -pitch_pid_speed_.value(), 0, 0);
}

/**
 * @brief 云台比赛下的pid解算
 * @tparam
 */
void Gimbal::GimbalMatchUpdate() {
  DaMiaoMotorEnable();
  GimbalRCDataUpdate();
  RotorMotorDataUpdate();

  if ((remote->key(RcKey::kF) == 1 || (rc_remote.data().keyboard_key >> 9 & 0x01) == 1) && XFstate_ == 0) {
    DFflag_ = 1;
  } else if (DFflag_ == 1) {
    DFflag_ = 0;
    DFstate_ ^= 1;
  }
  if ((remote->key(RcKey::kG) == 1 || (rc_remote.data().keyboard_key >> 10 & 0x01) == 1) && DFstate_ == 0) {
    XFflag_ = 1;
  } else if (XFflag_ == 1) {
    XFflag_ = 0;
    XFstate_ ^= 1;
  }

  if (DFstate_ == 1) {
    gimbal_imu_frame.mode = 0x04;  // 小符模式
  } else if (XFstate_ == 1) {
    gimbal_imu_frame.mode = 0x08;  // 大符模式
  } else {
    gimbal_imu_frame.mode = 0x01;  // 自瞄模式
  }

  if (gimbal_imu_frame.mode == 0x01 && ((aimbot_frame.AimbotState >> 0) & 0x01) == 1 &&
      aimbot_frame.AimbotTarget != 0x20 &&
      (remote->mouse_button_right() == 1 || rc_remote.data().mouse_button_right == 1)) {
    AimbotDateUpdate();
    AimPIDUpdate();
    yaw_motor_.SetCurrent(static_cast<i16>(aimbot_yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
    pitch_motor->SetPosition(0, 0, -aimbot_pitch_pid_speed_.value(), 0, 0);
  } else if (gimbal_imu_frame.mode == 0x04 && ((aimbot_frame.AimbotState >> 0) & 0x01) == 1) {
    AimbotDateUpdate();
    BuffPIDUpdate();
    yaw_motor_.SetCurrent(static_cast<i16>(buff_yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
    pitch_motor->SetPosition(0, 0, -buff_pitch_pid_speed_.value(), 0, 0);
  } else if (gimbal_imu_frame.mode == 0x08 && ((aimbot_frame.AimbotState >> 0) & 0x01) == 1) {
    AimbotDateUpdate();
    BuffPIDUpdate();
    yaw_motor_.SetCurrent(static_cast<i16>(buff_yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
    pitch_motor->SetPosition(0, 0, -buff_pitch_pid_speed_.value(), 0, 0);
  } else {
    MovePIDUpdate();
    yaw_motor_.SetCurrent(static_cast<i16>(yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
    pitch_motor->SetPosition(0, 0, -pitch_pid_speed_.value(), 0, 0);
  }
}

/**
 * @brief 云台自瞄下的pid解算
 * @tparam
 */
void Gimbal::GimbalAimbotUpdate() {
  DaMiaoMotorEnable();
  GimbalRCDataUpdate();
  AimbotDateUpdate();
  AimPIDUpdate();
  RotorMotorDataUpdate();
  yaw_motor_.SetCurrent(static_cast<i16>(aimbot_yaw_pid_speed_.value() + kyaw_current_ * yaw_motor_.rpm()));
  // osDelay(1000);
  pitch_motor->SetPosition(0, 0, -aimbot_pitch_pid_speed_.value(), 0, 0);
}

/**
 * @brief 云台无力下的pid解算
 * @tparam
 */
void Gimbal::GimbalDisableUpdate() {
  DaMiaoMotorDisable();
  gimbal_imu_frame.mode = 0x00;
  gimbal_yaw_rc_ = INS.yaw;  // 云台位置初始化
  gimbal_pitch_rc_ = INS.pitch;
  yaw_motor_.SetCurrent(0);
}

/**
 * @brief 发射机构有力下的pid解算
 * @tparam
 */
void Gimbal::AmmoEnableUpdate() {
  ammo_pid_left_.Update(-ammo_init_speed_ - ChassisRequestStatePacket_.AimSpeedChange * kammo_speed_change_,
                        ammo_left_.rpm());
  ammo_pid_right_.Update(ammo_init_speed_ + ChassisRequestStatePacket_.AimSpeedChange * kammo_speed_change_,
                         ammo_right_.rpm());
  ammo_left_.SetCurrent(ammo_pid_left_.value());
  ammo_right_.SetCurrent(ammo_pid_right_.value());
  ammo_left_speed_ = ammo_pid_left_.value();
  ammo_right_speed_ = ammo_pid_right_.value();
}

/**
 * @brief 发射机构无力下的pid解算
 * @tparam
 */
void Gimbal::AmmoDisableUpdate() {
  ammo_pid_left_.Update(0, ammo_left_.rpm());
  ammo_pid_right_.Update(0, ammo_right_.rpm());
  ammo_left_.SetCurrent(ammo_pid_left_.value());
  ammo_right_.SetCurrent(ammo_pid_right_.value());
}

/**
 * @brief 自瞄模式下，拨盘模式更新
 * @tparam
 */
void Gimbal::RotorAimbotUpdate() {
  if ((remote->dial() >= 650 || remote->mouse_button_left() == 1 || rc_remote.data().mouse_button_left == 1) &&
      (remote->mouse_button_right() == 1 || rc_remote.data().mouse_button_right == 1) &&
      ((aimbot_frame.AimbotState >> 1) & 0x01) == 1) {
    ammo_flag_rc_ = 1;
  } else if (remote->dial() >= 650 || remote->mouse_button_left() == 1 || rc_remote.data().mouse_button_left == 1) {
    ammo_flag_rc_ = 1;
  } else if (remote->dial() <= -650) {
    shoot_flag_ = 1;
  } else if (ammo_flag_rc_ == 1) {
    ammo_flag_rc_ = 0;
  } else if (ammo_flag_rc_ == 2) {
    shoot_flag_ = 0;
  } else {
    ammo_flag_rc_ = 0;
  }
  if ((rotor_target_position_ - rotor_position_) < rotor_position_dalte_ && ammo_flag_rc_ == 2) {
    ammo_flag_rc_ = 0;
  }
  if (shoot_flag_last_ == 0 && shoot_flag_ == 1) {
    rotor_target_position_ = rotor_position_ + once_circle_;
    ammo_flag_rc_ = 2;
  }

  if (ammo_flag_rc_ == 1 && back_turn_flag_ != 0 && abs(rotor_motor_.rpm()) < 1.0f) {
    back_turn_flag_--;
    if (back_turn_flag_ == 0) {
      back_turn_flag_ = 50;
    }
  } else if (back_turn_time_ == 0) {
    back_turn_flag_ = 100;
  }
  if (back_turn_time_ > 1 && back_turn_flag_ == 0) {
    ammo_flag_rc_ = -1;
    back_turn_time_--;
  } else if (back_turn_time_ == 1) {
    back_turn_time_ = 0;
    ammo_flag_rc_ = 0;
  }
}

/**
 * @brief 比赛模式下，拨盘模式更新
 * @tparam
 */
void Gimbal::RotorMatchUpdate() {
  if (gimbal_imu_frame.mode == 0x01) {
    if ((remote->dial() >= 650 || remote->mouse_button_left() == 1 || rc_remote.data().mouse_button_left == 1) &&
        (remote->mouse_button_right() == 1 || rc_remote.data().mouse_button_right == 1) &&
        ((aimbot_frame.AimbotState >> 1) & 0x01) == 1) {
      ammo_flag_rc_ = 1;
    } else if (remote->mouse_button_right() == 0 && rc_remote.data().mouse_button_right == 0 &&
               (remote->dial() >= 650 || remote->mouse_button_left() == 1 || rc_remote.data().mouse_button_left == 1)) {
      ammo_flag_rc_ = 1;
    } else if (remote->dial() <= -650) {
      shoot_flag_ = 1;
    } else if (ammo_flag_rc_ == 1) {
      ammo_flag_rc_ = 0;
    } else if (ammo_flag_rc_ == 2) {
      shoot_flag_ = 0;
    } else {
      ammo_flag_rc_ = 0;
    }
    if ((rotor_target_position_ - rotor_position_) < rotor_position_dalte_ && ammo_flag_rc_ == 2) {
      ammo_flag_rc_ = 0;
    }
    if (shoot_flag_last_ == 0 && shoot_flag_ == 1) {
      rotor_target_position_ = rotor_position_ + once_circle_;
      ammo_flag_rc_ = 2;
    }
  } else if (gimbal_imu_frame.mode == 0x04 || gimbal_imu_frame.mode == 0x08) {
    if (remote->mouse_button_left() == 1 || rc_remote.data().mouse_button_left == 1) {
      shoot_flag_ = 1;
    } else if (((aimbot_frame.AimbotState >> 1) & 0x01) == 1 &&
               (remote->mouse_button_right() == 1 || rc_remote.data().mouse_button_right == 1)) {
      shoot_flag_ = 1;
    } else {
      shoot_flag_ = 0;
    }
    if ((rotor_target_position_ - rotor_position_) < rotor_position_dalte_ && ammo_flag_rc_ == 2) {
      ammo_flag_rc_ = 0;
    }
    if (shoot_flag_last_ == 0 && shoot_flag_ == 1) {
      rotor_target_position_ = rotor_position_ + once_circle_;
      ammo_flag_rc_ = 2;
    }
  }
  shoot_flag_last_ = shoot_flag_;

  if (ammo_flag_rc_ == 1 && back_turn_flag_ != 0 && abs(rotor_motor_.rpm()) < 1.0f) {
    back_turn_flag_--;
    if (back_turn_flag_ == 0) {
      back_turn_flag_ = 50;
    }
  } else if (back_turn_time_ == 0) {
    back_turn_flag_ = 100;
  }
  if (back_turn_time_ > 1 && back_turn_flag_ == 0) {
    ammo_flag_rc_ = -1;
    back_turn_time_--;
  } else if (back_turn_time_ == 1) {
    back_turn_time_ = 0;
    ammo_flag_rc_ = 0;
  }
}

/**
 * @brief 无力模式下，拨盘数据更新
 * @tparam
 */
void Gimbal::RotorDisableUpdate() {
  rotor_pid_speed_.Update(0, rotor_motor_.rpm());
  rotor_motor_.SetCurrent(0);
  rotor_position_ = 0.0f;
  rotor_target_position_ = 0.0f;
  rotor_circle_flag_ = 0;
}

/**
 * @brief 拨盘模式更新
 * @tparam
 */
void Gimbal::RotorEnableUpdate() {  // 没有单发模式，即时射击，无倒计时
  switch ((StateMachine_)) {
    case GM_MATCH:
      RotorMatchUpdate();
      break;

    case GM_AIMBOT:
      RotorAimbotUpdate();
      break;

    default:
      ammo_flag_rc_ = 0;
      break;
  }

  // 接收热量信息
  heat_limit_ = chassis_communicator->heat_limit();
  heat_real_ = chassis_communicator->heat_real();

  // 热量延时检测
  if (heat_last_ != heat_real_) {
    heat_last_ = heat_real_;
    heat_delaytime_ = 0;
  } else if (heat_last_ == heat_real_ && heat_last_ != 0) {
    heat_delaytime_++;
  } else if (heat_real_ == 0) {
    heat_delaytime_ = 0;
  }
  if ((ammo_flag_rc_ == 1) && (heat_limit_ - heat_real_ - ((heat_delaytime_) / 2) >= 20)) {
    if (heat_limit_ < 100) {
      rotor_pid_speed_.Update(rotor_init_speed_[0], rotor_motor_.rpm());
      rotor_motor_.SetCurrent(rotor_pid_speed_.value());
    } else {
      rotor_pid_speed_.Update(rotor_init_speed_[1], rotor_motor_.rpm());
      rotor_motor_.SetCurrent(rotor_pid_speed_.value());
    }
  } else if (ammo_flag_rc_ == 2) {
    rotor_pid_speed_.Update(rotor_init_speed_[2], rotor_motor_.rpm());
    rotor_motor_.SetCurrent(rotor_pid_speed_.value());
  } else if (ammo_flag_rc_ == -1) {
    rotor_pid_speed_.Update(rotor_init_speed_[3], rotor_motor_.rpm());
    rotor_motor_.SetCurrent(rotor_pid_speed_.value());
  } else {
    rotor_pid_speed_.Update(0, rotor_motor_.rpm());
    rotor_motor_.SetCurrent(rotor_pid_speed_.value());
  }
}

/**
 * @brief 达妙电机使能
 * @tparam
 */
void Gimbal::DaMiaoMotorEnable() {
  if (DMEnable_ == 0) {
    // osDelay(0.1);
    pitch_motor->SendInstruction(DmMotorInstructions::kEnable);  // 使达妙电机使能
    DMEnable_ = 1;
  }
}

/**
 * @brief 达妙电机失能
 * @tparam
 */
void Gimbal::DaMiaoMotorDisable() {
  if (DMEnable_ == 1) {
    // osDelay(0.1);
    pitch_motor->SendInstruction(DmMotorInstructions::kDisable);  // 使达妙电机失能
    DMEnable_ = 0;
  }
}

/**
 * @brief 遥控数据更新
 * @tparam
 */
void Gimbal::GimbalRCDataUpdate() {
  gimbal_yaw_rc_ -= Map(
      remote->left_x() + remote->mouse_x() * kmouse_sensitivity_x_ + rc_remote.data().mouse_x * kmouse_sensitivity_x_,
      -660, 660, -sensitivity_x_, sensitivity_x_);
  gimbal_pitch_rc_ -= Map(
      remote->left_y() + remote->mouse_y() * kmouse_sensitivity_y_ + rc_remote.data().mouse_y * kmouse_sensitivity_y_,
      -660, 660, -sensitivity_y_, sensitivity_y_);
}

/**
 * @brief 自瞄云台目标值更新
 * @tparam
 */
void Gimbal::AimbotDateUpdate() {
  if ((aimbot_frame.AimbotState >> 0) & 0x01) {
    gimbal_yaw_rc_ = aimbot_frame.Yaw;
    gimbal_pitch_rc_ = aimbot_frame.Pitch;
  }
}

/**
 * @brief 运动pid数据更新
 * @tparam
 */
void Gimbal::MovePIDUpdate() {
  gimbal_yaw_rc_ = LoopConstrain(gimbal_yaw_rc_, 0.0f, 360.0f);                                // yaw轴周期限制
  gimbal_pitch_rc_ = Constrain(gimbal_pitch_rc_, -highest_pitch_angle_, lowest_pitch_angle_);  // pitch轴限位
  yaw_pid_position_.Update(gimbal_yaw_rc_, INS.yaw);                                           // pid更新
  yaw_pid_speed_.Update(yaw_pid_position_.value() + kyaw_speed_ * yaw_motor_.rpm(), yaw_motor_.rpm());
  pitch_pid_position_.Update(-gimbal_pitch_rc_, -INS.pitch);
  pitch_pid_speed_.Update(pitch_pid_position_.value(), pitch_motor->vel());
}

/**
 * @brief 自瞄pid数据更新
 * @tparam
 */
void Gimbal::AimPIDUpdate() {
  gimbal_yaw_rc_ = LoopConstrain(gimbal_yaw_rc_, 0.0f, 360.0f);                                // yaw轴周期限制
  gimbal_pitch_rc_ = Constrain(gimbal_pitch_rc_, -highest_pitch_angle_, lowest_pitch_angle_);  // pitch轴限位

  aimbot_yaw_pid_position_.Update(gimbal_yaw_rc_, INS.yaw);  // pid更新
  aimbot_yaw_pid_speed_.Update(aimbot_yaw_pid_position_.value() + kyaw_speed_ * yaw_motor_.rpm(), yaw_motor_.rpm());
  aimbot_pitch_pid_position_.Update(-gimbal_pitch_rc_, -INS.pitch);
  aimbot_pitch_pid_speed_.Update(aimbot_pitch_pid_position_.value(), pitch_motor->vel());
}

/**
 * @brief 打符pid数据更新
 * @tparam
 */
void Gimbal::BuffPIDUpdate() {
  gimbal_yaw_rc_ = LoopConstrain(gimbal_yaw_rc_, 0.0f, 360.0f);                                // yaw轴周期限制
  gimbal_pitch_rc_ = Constrain(gimbal_pitch_rc_, -highest_pitch_angle_, lowest_pitch_angle_);  // pitch轴限位

  buff_yaw_pid_position_.Update(gimbal_yaw_rc_, INS.yaw);  // pid更新
  buff_yaw_pid_speed_.Update(buff_yaw_pid_position_.value() + kyaw_speed_ * yaw_motor_.rpm(), yaw_motor_.rpm());
  buff_pitch_pid_position_.Update(-gimbal_pitch_rc_, -INS.pitch);
  buff_pitch_pid_speed_.Update(buff_pitch_pid_position_.value(), pitch_motor->vel());
}

/**
 * @brief 拨盘电机数据更新
 * @tparam
 */
void Gimbal::RotorMotorDataUpdate() {
  if ((last_rotor_position_ - rotor_motor_.encoder()) >= 5000.f) {  // 过圈检测
    rotor_circle_flag_++;
  } else if ((last_rotor_position_ - rotor_motor_.encoder()) <= -5000.f) {
    rotor_circle_flag_--;
  }
  rotor_position_ = rotor_motor_.encoder() + 8191.f * rotor_circle_flag_;
  last_rotor_position_ = rotor_motor_.encoder();
}

extern "C" {
void GimbalTask(void const *argument) {
  gimbal = new Gimbal();
  gimbal->GimbalInit();
  while (1) {
    gimbal->StateUpdate();         // 云台状态更新
    gimbal->GimbalUpdate();        // 云台所有电机状态更新
    gimbal->ChassisStateUpdate();  // 底盘状态更新
    DjiMotor<>::SendCommand();     // 发送电流

    osDelay(1);
  }
}
}