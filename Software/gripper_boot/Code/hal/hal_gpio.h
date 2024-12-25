#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED_Pin 									GPIO_PIN_0
#define LED_GPIO_Port 						GPIOA

#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOB

void __hal_gpio_init(void);
void __hal_gpio_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t mode);
void __hal_gpio_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void __hal_gpio_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState __hal_gpio_read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

