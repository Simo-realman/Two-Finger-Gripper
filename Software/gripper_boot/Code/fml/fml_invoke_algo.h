#ifndef __FML_ALGO_H
#define __FML_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "common.h"

void fml_my_algo_init(int _pos_now, int _pos_tag, uint8_t _speed);
uint8_t fml_my_algo_plan(int *pPos);
	
#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
