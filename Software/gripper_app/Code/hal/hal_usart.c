#include "hal_usart.h"

//#include "stdio.h"
//#include "string.h"
//#include "queue.h"

void (*uart1RecvCb)(uint8_t *, uint16_t);

#define UART_DMA_BUF_LEN 100

__align(4) uint8_t uart1_Data_DMA_Buffer[UART_DMA_BUF_LEN] = {0};
__align(4) uint8_t uart3_Data_DMA_Buffer[UART_DMA_BUF_LEN] = {0};

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

//extern TRANS_QUEUE qUart1Recive;

void __hal_uart1_init(e_baud_type baud, void (*_cb)(uint8_t *, uint16_t))
{
	uart1RecvCb = _cb;
	
	if(baud == E_BAUD_9600)
	{
		huart1.Init.BaudRate = 9600;
	}
	else if(baud == E_BAUD_19200)
	{
		huart1.Init.BaudRate = 19200;
	}
	else if(baud == E_BAUD_38400)
	{
		huart1.Init.BaudRate = 38400;
	}
	else if(baud == E_BAUD_57600)
	{
		huart1.Init.BaudRate = 57600;
	}
	else if(baud == E_BAUD_115200)
	{
		huart1.Init.BaudRate = 115200;
	}
	else if(baud == E_BAUD_460800)
	{
		huart1.Init.BaudRate = 460800;
	}
	else 
	{
		huart1.Init.BaudRate = 115200;
	}
	HAL_UART_DeInit(&huart1);
	HAL_Delay(10);
  huart1.Instance = USART1;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	HAL_UART_Receive_DMA(&huart1,uart1_Data_DMA_Buffer,UART_DMA_BUF_LEN);
}

void __hal_uart3_init(void)
{
	HAL_UART_DeInit(&huart3);
	HAL_Delay(10);
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,uart3_Data_DMA_Buffer,UART_DMA_BUF_LEN);
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* USART1 interrupt Init */
    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel3;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);
		/* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel2;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);
		/* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */
	
  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
		
		HAL_DMA_DeInit(uartHandle->hdmarx);
		
		HAL_NVIC_DisableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
		
		HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

void hal_uart1_send(uint8_t *data,uint8_t len)
{
	UART1_DE(1);
  HAL_UART_Transmit(&huart1,data,len,1000);
  while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);
	UART1_DE(0);
}

void hal_uart3_send(uint8_t *data,uint8_t len)
{
	UART3_DE(1);
  HAL_UART_Transmit(&huart3,data,len,1000);
  while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)!=SET);
	UART3_DE(0);
}
volatile int fouce[6];
//int u8Arry2float(uint8_t *data, bool key)
//{
//	float fa = 0;
//	uint8_t uc[4] = {0};
//	if(key)
//	{
//		uc[3] = data[0];
//		uc[2] = data[1];
//		uc[1] = data[2];
//		uc[0] = data[3];
//	}
//	else
//	{
//		uc[0] = data[0];
//		uc[1] = data[1];
//		uc[2] = data[2];
//		uc[3] = data[3];
//	}
//	memcpy(&fa,uc,4);
//	return (int)(fa * 9810);
//}

int recive_len = 0;
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
	
	uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);
	if((tmp_flag != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		recive_len = UART_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		if(recive_len != 0)
		{
			uart1RecvCb(uart1_Data_DMA_Buffer, recive_len);
		}
		if(HAL_OK != HAL_UART_Receive_DMA(&huart1,uart1_Data_DMA_Buffer,UART_DMA_BUF_LEN))
		{
		}
	}
	else
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		HAL_UART_Receive_DMA(&huart1,uart1_Data_DMA_Buffer,UART_DMA_BUF_LEN);
	}
}

void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart3);
	
	uint8_t tmp_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); 
	//union force union_force = {0};
	if((tmp_flag != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		HAL_UART_DMAStop(&huart3);
		int uart3_recive_len = UART_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		HAL_UART_Receive_DMA(&huart3,uart3_Data_DMA_Buffer,UART_DMA_BUF_LEN);
	}
	else
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		HAL_UART_DMAStop(&huart3);
		HAL_UART_Receive_DMA(&huart3,uart3_Data_DMA_Buffer,UART_DMA_BUF_LEN);
	}
}
/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		__HAL_UNLOCK(huart);
		__HAL_UART_CLEAR_OREFLAG(huart);
//		HAL_UART_Receive_IT(&huart1,Rx485_buf,1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1,uart1_Data_DMA_Buffer,UART_DMA_BUF_LEN);
	}
	if(huart->Instance==USART2)
	{
		__HAL_UNLOCK(huart);
		__HAL_UART_CLEAR_OREFLAG(huart);
//		HAL_UART_Receive_IT(&huart2,Rx232_buf,1);
	}
	if(huart->Instance==USART3)
	{
		__HAL_UNLOCK(huart);
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart3,uart1_Data_DMA_Buffer,UART_DMA_BUF_LEN);
//		HAL_UART_Receive_IT(&huart3,USART3_buf,1);
	}
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
