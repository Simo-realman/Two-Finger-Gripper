#ifndef __FML_ALGO_H
#define __FML_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "common.h"

#define HOME_THETA 					0	
#define PLAN_CYCLE_MS 			2 //规划周期	
	
void fml_algo_stop_init(void);//急停规划
void fml_algo_plan_init(uint16_t _pos_tag, uint16_t _speed, uint16_t _force); //运动规划
void fml_filter_init(void);
void fml_algo_init(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
