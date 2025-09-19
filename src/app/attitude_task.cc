#include "attitude_task.hpp"

using rm::device::BMI088;
using rm::modules::algorithm::MahonyAhrs;

INS_t INS;

extern "C"
{
    void AttitudeTask(void const *argument)
    {
        vTaskDelay(30);
        // rm::modules::algorithm::ImuData6Dof bmi_data; // 储存bmi数据
        BMI088 bmi088(hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port,
                      CS1_GYRO_Pin); // 初始化bmi
        MahonyAhrs mahony(1000.0f);  // 初始化 mahony

        for (;;)
        {
            // 更新数据
            bmi088.Update();
            mahony.Update(rm::modules::algorithm::ImuData6Dof{-bmi088.gyro_y(), bmi088.gyro_x(), bmi088.gyro_z() + INS.yaw_gyro_bias,
                                                              -bmi088.accel_y(), bmi088.accel_x(), bmi088.accel_z()});

            INS.q[0] = mahony.quaternion().w;
            INS.q[1] = mahony.quaternion().x;
            INS.q[2] = mahony.quaternion().y;
            INS.q[3] = mahony.quaternion().z;

            INS.yaw = 57.3 * mahony.euler_angle().roll;
            INS.roll = 57.3 * mahony.euler_angle().yaw;
            INS.pitch = 57.3 * mahony.euler_angle().pitch;

            INS.gyro[0] = -bmi088.gyro_y();
            INS.gyro[1] = bmi088.gyro_x();
            INS.gyro[2] = bmi088.gyro_z() + 0.0015f;

            INS.accel[0] = -bmi088.accel_y();
            INS.accel[1] = bmi088.accel_x();
            INS.accel[2] = bmi088.accel_z();
            osDelay(1);
        }
    }
}
