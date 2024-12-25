#ifndef __APL_MAIN_TASK_H
#define __APL_MAIN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void peripheral_init(void);
void fdcan_send_task_entry(void);
void uart_send_task_entry(void);
void uart_data_parse_task_entry(void);
void fdcan_data_parse_task_entry(void);
void state_update_task_entry(void);
void force_postion_task_entry(void);
#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
