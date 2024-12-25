#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
	E_BAUD_9600 = 0,
	E_BAUD_19200   ,
	E_BAUD_38400	 ,
	E_BAUD_57600	 ,
	E_BAUD_115200	 ,
	E_BAUD_460800	 ,
}e_baud_type;

#define UART1_DE_Pin GPIO_PIN_5
#define UART1_DE_GPIO_Port GPIOB
#define UART3_DE_Pin GPIO_PIN_15
#define UART3_DE_GPIO_Port GPIOA

#define UART1_DE(n)   (n?HAL_GPIO_WritePin(UART1_DE_GPIO_Port,UART1_DE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(UART1_DE_GPIO_Port,UART1_DE_Pin,GPIO_PIN_RESET))
#define UART3_DE(n)   (n?HAL_GPIO_WritePin(UART3_DE_GPIO_Port,UART3_DE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(UART3_DE_GPIO_Port,UART3_DE_Pin,GPIO_PIN_RESET))

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void __hal_uart1_init(e_baud_type baud, void (*_cb)(uint8_t *, uint16_t));
void __hal_uart3_init(void);

void hal_uart1_send(uint8_t *data,uint8_t len);
void hal_uart3_send(uint8_t *data,uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

