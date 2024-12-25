#include "hal_gpio.h"
#include "hal_usart.h"

s_io_state g_dio_state[2] = {{E_DIO_MODE_OUT, E_DIO_LOW},{E_DIO_MODE_OUT, E_DIO_LOW}}; //DIO状态

void __hal_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
	fml_set_rgb(E_RGB_BLUE);
	//HAL_GPIO_WritePin(LED_GPIO_Port, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
	
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
	
	__hal_gpio_mode(DIN1_GPIO_Port, DIN1_Pin, MODE_INPUT);
	__hal_gpio_mode(DIN2_GPIO_Port, DIN2_Pin, MODE_INPUT);
	__hal_gpio_mode(OUT1_GPIO_Port, OUT1_Pin, GPIO_MODE_OUTPUT_PP);
	__hal_gpio_mode(OUT2_GPIO_Port, OUT2_Pin, GPIO_MODE_OUTPUT_PP);
	
	HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,GPIO_PIN_RESET); //输入模式-默认拉高
	HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,GPIO_PIN_RESET); //输入模式-默认拉高
	
	/*
	IO输出功能说明：
	DOUT输出高电平，ULN2001D三极管导通拉低，对外功能输出低电平。
	DOUT输出低电平，ULN2001D三极管不导通，对外功能输出高电平。
	
	IO输入功能说明：
	DOUT输出低电平，对外默认拉高，此时可做输入模式。
	*/
//	DOUT1(0); //输入模式
//	DOUT2(0); //输入模式
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
	if(mode == MODE_INPUT)
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	else
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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



void fml_set_rgb(e_rgb rgb)
{
	if(rgb == E_RGB_RED)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_GREEN_PIN|RGB_BLUE_PIN, GPIO_PIN_SET);
	}
	else if(rgb == E_RGB_BLUE)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_BLUE_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN|RGB_GREEN_PIN, GPIO_PIN_SET);
	}
	else if(rgb == E_RGB_GREEN)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_GREEN_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN|RGB_BLUE_PIN, GPIO_PIN_SET);
	}
	else if(rgb == E_RGB_WHITE)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN|RGB_GREEN_PIN|RGB_BLUE_PIN, GPIO_PIN_RESET);
	}
	else if(rgb == E_RGB_YELLOW)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_BLUE_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN|RGB_GREEN_PIN, GPIO_PIN_RESET);
	}
	else if(rgb == E_RGB_VIOLET)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_GREEN_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_GPIO_Port, RGB_RED_PIN|RGB_BLUE_PIN, GPIO_PIN_RESET);
	}
}
