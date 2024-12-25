#include "fml_storage.h"
#include "hal_flash.h"
#include "hal_usart.h"
#include "hal_gpio.h"

#define DBG_TAG "storage."
#define DBG_LVL DBG_ERROR
#include "apl_dbg.h"

system_parameters_t spt;
extern s_io_state g_dio_state[2];

void fml_system_param_default(void)
{
	spt.valid = 0xFE;
	spt.device_id = 0x01;
	spt.uart1_baud = E_BAUD_115200;
	spt.upgrade_en = 0;
	
	spt.pos_limit_min = 0; //0-1000
	spt.pos_limit_max = 1000; //0-1000
	
	spt.servo_speed = 500; //0-1000
	spt.servo_force = 500; //0-1000
	
	spt.motor_id = 0x01;
	spt.key_enable = 0;
	
	spt.DIO1_mode = E_DIO_MODE_IN; //IO1模式
	spt.DIO1_state = 1; //IO1状态
	spt.DIO2_mode = E_DIO_MODE_IN; //IO2模式
	spt.DIO2_state = 1; //IO2状态
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
			if(fml_storage_save(E_DATA_ALL) != RM_OK) 
			{
				fml_storage_save(E_DATA_ALL);
			}
		}
		else
		{
			if(spt.upgrade_en != 0) //复位升级状态
			{
				spt.upgrade_en = 0;
				if(fml_storage_save(E_DATA_ALL) != RM_OK) 
				{
					fml_storage_save(E_DATA_ALL);
				}
			}
		}
	}
	else
	{
		NVIC_SystemReset(); //重启
	}
}

void fml_system_param_set(system_parameters_t *_param)
{
	spt.valid = _param->valid;
	spt.device_id = _param->device_id;
	spt.uart1_baud = _param->uart1_baud;
	
	spt.pos_limit_min = _param->pos_limit_min;
	spt.pos_limit_max = _param->pos_limit_max;
	spt.servo_speed = _param->servo_speed;
	spt.servo_force = _param->servo_force;
}

//读flash
system_parameters_t *fml_storage_get(e_apt_flag flag)
{
	if(flag == READ_FLASH)
	{
		if(RM_OK != hal_flash_read_buf(STORAGE_BASE_ADDR, (uint8_t *)&spt, sizeof(spt)))
		{
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
		spt.DIO1_mode = g_dio_state[0].pin_mode; //IO1模式
		spt.DIO1_state = DIN1; //IO1状态
		spt.DIO2_mode = g_dio_state[1].pin_mode; //IO2模式
		spt.DIO2_state = DIN2; //IO2状态

		hal_flash_erase(62, 100);
		return hal_flash_write_buf(STORAGE_BASE_ADDR, (uint8_t *)&spt, sizeof(spt));
	}
	
	return RM_OK;
}

