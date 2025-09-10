#ifndef ATTITUDE_TASK_H
#define ATTITUDE_TASK_H

#include "cmsis_os.h"

#include "librm.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    extern void attitude_task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_TASK_H */