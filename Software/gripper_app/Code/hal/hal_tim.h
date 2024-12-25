#ifndef __HAL_TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim2;

void __hal_timer2_init(void);
//void __hal_timer2_init(void (*_cb)(void));

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

