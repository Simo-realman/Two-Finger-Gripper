#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED_Pin 									GPIO_PIN_11
#define LED_GPIO_Port 						GPIOA
	
#define DIN1_Pin 								GPIO_PIN_14
#define DIN1_GPIO_Port 					GPIOC
#define DIN2_Pin 								GPIO_PIN_10
#define DIN2_GPIO_Port 					GPIOB

#define OUT1_Pin 								GPIO_PIN_12
#define OUT1_GPIO_Port 					GPIOB
#define OUT2_Pin 								GPIO_PIN_11
#define OUT2_GPIO_Port 					GPIOB	
	
#define DIN1           (HAL_GPIO_ReadPin(DIN1_GPIO_Port,DIN1_Pin))
#define DIN2           (HAL_GPIO_ReadPin(DIN2_GPIO_Port,DIN2_Pin))	
	
#define DOUT1(n)       (n?HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET))
#define DOUT2(n)       (n?HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET))
	
#define RGB_RED_PIN  		GPIO_PIN_10
#define RGB_GREEN_PIN   GPIO_PIN_11
#define RGB_BLUE_PIN  	GPIO_PIN_12

typedef enum {
	E_RGB_RED = 0,
	E_RGB_BLUE,
	E_RGB_GREEN,
	E_RGB_WHITE,
	E_RGB_YELLOW,
	E_RGB_VIOLET,
}e_rgb;

enum {
	E_DIO_MODE_IN = 0,
	E_DIO_MODE_OUT 	 ,
};

enum {
	E_DIO_LOW = 0,
	E_DIO_HIGH 	 ,
};
	
typedef struct {
	uint8_t pin_mode;
	uint8_t pin_state;
}s_io_state; 	
	
void __hal_gpio_init(void);
void __hal_gpio_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t mode);
void __hal_gpio_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void __hal_gpio_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState __hal_gpio_read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void fml_set_rgb(e_rgb rgb);

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

