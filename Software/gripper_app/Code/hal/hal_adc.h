#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern ADC_HandleTypeDef hadc2;
void hal_adc2_init(void);
void hal_adc2_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

