#include "hal_iwdg.h"
#include "stm32g4xx_hal_iwdg.h"

IWDG_HandleTypeDef hiwdg;

void __hal_iwdg_init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 1500;
  hiwdg.Init.Reload = 1500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void __hal_iwdg_reload(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}
