#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "cmsis_os.h"

#include "librm.hpp"

#ifdef __cplusplus
extern "C" {
#endif

extern void communicate_task(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* COMMUNICATE_TASK_H */