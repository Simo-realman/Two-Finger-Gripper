#include "fml_storage.h"
#include "hal_flash.h"
#include "hal_usart.h"

#define DBG_TAG "storage."
#define DBG_LVL DBG_ERROR
#include "apl_dbg.h"

system_parameters_t spt;

void fml_system_param_default(void)
{
	//spt.valid = 0xFE;
	spt.device_id = 0x01;
	spt.uart1_baud = E_BAUD_115200;
	spt.upgrade_en = 0; //不升级
}

system_parameters_t *SYSTEM_PARAM(void)
{
	return &spt;
}

void fml_system_param_init(void)
{
	if(fml_storage_get(READ_FLASH) != NULL)
	{
		if(spt.valid != 0xFE) //初始化参数
		{
			fml_system_param_default(); //使用默认参数
//			if(fml_storage_save(E_DATA_ALL) != RM_OK) 
//			{
//				fml_storage_save(E_DATA_ALL);
//			}
		}
	}
	else
	{
		NVIC_SystemReset(); //重启
	}
}

//读flash
system_parameters_t *fml_storage_get(e_apt_flag flag)
{
	if(flag == READ_FLASH)
	{
		if(RM_OK != hal_flash_read_buf(STORAGE_BASE_ADDR, (uint8_t *)&spt, sizeof(spt)))
		{
			//LOG_E("err. get system param\r\n");
			return NULL;
		}
	}
	return &spt;
}
//写flash
rm_result_e fml_storage_save(uint32_t items)
{
	if(items == E_DATA_ALL)
	{
		hal_flash_erase(62, 100);
		return hal_flash_write_buf(STORAGE_BASE_ADDR, (uint8_t *)&spt, sizeof(spt));
	}
	
	return RM_OK;
}

