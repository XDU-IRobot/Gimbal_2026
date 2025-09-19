#ifndef ATTITUDE_TASK_H
#define ATTITUDE_TASK_H

#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"

#include "librm.hpp"

using namespace rm;

struct INS_t {
  f32 q[4];  // 四元数

  f32 yaw;    // 偏航角（角度值）
  f32 pitch;  // 俯仰角（角度值）
  f32 roll;   // 横滚角（角度值）

  f32 gyro[3];   // 角速度
  f32 accel[3];  // 加速度

  const f32 yaw_gyro_bias = 0.0015f;  // 偏航角（角度值）的陀螺仪偏移量
};

#ifdef __cplusplus
extern "C" {
#endif

extern void AttitudeTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_TASK_H */