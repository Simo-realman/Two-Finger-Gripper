#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;

void __hal_fdcan1_init(void (*_cb)(FDCAN_RxHeaderTypeDef *, uint8_t *));
uint8_t hal_fdcan1_send(uint32_t id, uint8_t* msg, uint32_t len);
uint32_t FDCANLEN_TO_LEN(uint32_t fdcan_len);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

