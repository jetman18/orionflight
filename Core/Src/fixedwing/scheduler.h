#ifndef _SCHEDULER_
#define _SCHEDULER_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

typedef struct {
    void (*exec)();
    uint32_t execution_time_us;
    uint32_t execution_cycle_us;
    uint32_t last_exec_time_us;
    uint32_t period;
} task_t;

void start_scheduler();
void init_sche();
#ifdef __cplusplus
}
#endif
#endif
