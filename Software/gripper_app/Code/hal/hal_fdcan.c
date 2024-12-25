#include "hal_fdcan.h"

extern uint32_t g_motor_can_id;
//void ctrl_data_parse(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData);

void (*canfdParseCb)(FDCAN_RxHeaderTypeDef *, uint8_t *);

FDCAN_HandleTypeDef hfdcan1;
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;

uint8_t rxdata[64];

void __hal_fdcan1_init(void (*_cb)(FDCAN_RxHeaderTypeDef *, uint8_t *))
{
	
	canfdParseCb = _cb;
	
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	/* USER CODE BEGIN FDCAN1_Init 2 */
	//配置RX滤波器
//	sFilterConfig.IdType = FDCAN_STANDARD_ID;                     //标准ID
//	sFilterConfig.FilterIndex = 0;                                //滤波器索引
//	sFilterConfig.FilterType = FDCAN_FILTER_MASK;                 //滤波器类型 FDCAN_FILTER_MASK  FDCAN_FILTER_DUAL   FDCAN_FILTER_RANGE 一加
//	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;         //过滤器0关联到FIFO0
//	sFilterConfig.FilterID1 = g_motor_can_id;                               	//32位ID
//	sFilterConfig.FilterID2 = 0x0ff;                              //如果FDCAN配置为传统模式的话，这里是32位掩码
//	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) //滤波器初始化
//	{
//			Error_Handler();
//	}
	//广播帧
	sFilterConfig.IdType = FDCAN_STANDARD_ID;                     //标准ID
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;                 //滤波器类型 FDCAN_FILTER_MASK  FDCAN_FILTER_DUAL   FDCAN_FILTER_RANGE 一加
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;         //过滤器0关联到FIFO0
	sFilterConfig.FilterIndex = 0;                                //滤波器索引
	sFilterConfig.FilterID1 = 0x000;                               	//32位ID
	sFilterConfig.FilterID2 = 0x000;                              //如果FDCAN配置为传统模式的话，这里是32位掩码
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) //滤波器初始化
	{
			Error_Handler();
	}
//下面这一句是配置全局滤波器配置寄存器的，一定要写，否则配置了也没用
///    HAL_FDCAN_ConfigGlobalFilter(&FDCAN1_Handler, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
	{
			Error_Handler();
	}
	///////////////////////////////////////////////////////////////////////////////////////////////		
	
	/* Prepare Tx image data message header */
	FDCAN1_TxHeader.Identifier = 0x001;
	FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
	FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_24;
	FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;
	FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	FDCAN1_TxHeader.MessageMarker = 0;

    /* Configure and enable Tx Delay Compensation : TdcOffset = DataTimeSeg1*DataPrescaler */

	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
	
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan1);
  /* USER CODE END FDCAN1_Init 2 */
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
	while(!HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, rxdata))
  ;
	canfdParseCb(&FDCAN1_RxHeader, rxdata);
}

uint8_t hal_fdcan1_send(uint32_t id, uint8_t* msg, uint32_t len)
{
	switch(len)
	{
		case 0:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_0;break;
		case 1:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_1;break;
		case 2:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_2;break;
		case 3:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_3;break;
		case 4:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_4;break;
		case 5:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_5;break;
		case 6:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_6;break;
		case 7:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_7;break;
		case 8:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_8;break;
		case 9:case 10:case 11:case 12:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_12;break;
		case 13:case 14:case 15:case 16:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_16;break;
		case 17:case 18:case 19:case 20:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_20;break;
		case 21:case 22:case 23:case 24:FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_24;break;
		default:
		{
			if(len <= 32)
				FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_32;
			else if(len <= 48)
				FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_48;
			else if(len <= 64)
				FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_64;
			else
				return 2;	//非法，直接退出
		}
	}
	FDCAN1_TxHeader.Identifier = id;                         //32位ID
	FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;                //标准ID
	FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;            //数据帧
	FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;             //打开速率切换
	FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;              //传统的CAN模式  FDCAN_FD_CAN  FDCAN_CLASSIC_CAN
	FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;   //无发送事件
	FDCAN1_TxHeader.MessageMarker = 0;
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, msg) != HAL_OK) return 1; //发送
	return 0;
}
/*
*	FDCAN的len转化为正常的长度
*/
uint32_t FDCANLEN_TO_LEN(uint32_t fdcan_len)
{
	switch(fdcan_len)
	{
		case FDCAN_DLC_BYTES_0:return 0;
		case FDCAN_DLC_BYTES_1:return 1;
		case FDCAN_DLC_BYTES_2:return 2;
		case FDCAN_DLC_BYTES_3:return 3;
		case FDCAN_DLC_BYTES_4:return 4;
		case FDCAN_DLC_BYTES_5:return 5;
		case FDCAN_DLC_BYTES_6:return 6;
		case FDCAN_DLC_BYTES_7:return 7;
		case FDCAN_DLC_BYTES_8:return 8;
		case FDCAN_DLC_BYTES_12:return 12;
		case FDCAN_DLC_BYTES_16:return 16;
		case FDCAN_DLC_BYTES_20:return 20;
		case FDCAN_DLC_BYTES_24:return 24;
		case FDCAN_DLC_BYTES_32:return 24;
		case FDCAN_DLC_BYTES_48:return 48;
		case FDCAN_DLC_BYTES_64:return 64;
		default:
		{
			return 0;
		}
	}
}
