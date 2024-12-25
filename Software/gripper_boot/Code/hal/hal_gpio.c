#include "hal_gpio.h"
#include "hal_usart.h"

void __hal_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
//	/*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = LED_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	
//  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = LED_G_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	
		/*Configure GPIO pins : UART1_DE */
  GPIO_InitStruct.Pin = UART1_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART1_DE_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(UART1_DE_GPIO_Port, UART1_DE_Pin, GPIO_PIN_RESET);
	
	/*Configure GPIO pins : UART3_DE */
  GPIO_InitStruct.Pin = UART3_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UART3_DE_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(UART3_DE_GPIO_Port, UART3_DE_Pin, GPIO_PIN_RESET);
}

void __hal_gpio_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void __hal_gpio_write(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

GPIO_PinState __hal_gpio_read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void __hal_gpio_toggle(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}




