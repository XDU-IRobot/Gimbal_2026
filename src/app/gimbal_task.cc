#include "gimbal_task.hpp"

using namespace rm;
using namespace rm::hal;
using namespace rm::device;
using namespace rm::modules::algorithm;
using namespace rm::modules::algorithm::utils;

static Gimbal *gimbal;
static VT03 *tcremote;
static DR16 *remote;
static hal::Serial *remote_uart;
static hal::Can can1(hcan1);
static hal::Can can2(hcan2);

Gimbal::Gimbal() : yaw_motor_(can1, 4),

                   ammo_left_(can2, 1),
                   ammo_right_(can2, 2),

                   rotor_motor_(can1, 3),

                   StateMachine_(Gimbal::StateMachineType::GM_NO_FORCE),

                   left_flag_(RcSwitchState::kDown),
                   right_flag_(RcSwitchState::kDown),

                   RequestStatePacket_{0, 0, 0, 0, 0, 0, 0, 0},

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

                   once_circle_(17000.0),

                   back_turn_time_(0),
                   back_turn_flag_(100),

                   rotor_position_(0.0),
                   last_rotor_position_(0.0),
                   rotor_target_position_(0.0),
                   rotor_circle_flag_(0),

                   shoot_flag_(0),
                   shoot_flag_last_(0),

                   kyaw_speed_(1.2),
                   kyaw_current_(-0.5),

                   speed_change_flag_(0),

                   single_wheel_flag_(0),
                   single_wheel_state_(0),

                   DMEnable_(0),

                   DFflag_(0),
                   XFflag_(0),
                   DFstate_(0),
                   XFstate_(0)
{
}

/**
 * @brief  初始化can,遥控器
 * @note
 * @tparam
 */
void Gimbal::GimbalInit()
{
    can1.SetFilter(0, 0);
    can2.SetFilter(0, 0);
    can1.Begin();
    can2.Begin();

    this->pitch_motor_settings_ = new DmMotorSettings<DmMotorControlMode::kMit>{
        0x03, 0x02, 12.5f, 30.0f, 10.0f, std::make_pair(0.0f, 500.0f), std::make_pair(0.0f, 5.0f)};
    this->pitch_motor_ = new DmMotor<DmMotorControlMode::kMit>(can2, *this->pitch_motor_settings_, true);

    remote_uart = new hal::Serial(huart3, 18, hal::stm32::UartMode::kNormal, hal::stm32::UartMode::kDma);
    remote = new DR16(*remote_uart);
    remote->Begin();

    gimbal = new Gimbal();

    // while (chassis_controler->gimbal_power_state_ == 0)
    // {
    //     DMEnable = 0; // 检测到云台上电才退出
    // }

    osDelay(1000);
    // gimbal->gimbal_yaw_rc = yaw;     // 云台yaw初始化
    // gimbal->gimbal_pitch_rc = pitch; // 云台pitch初始化
}

extern "C"
{
    void GimbalTask(void const *argument) {}
}