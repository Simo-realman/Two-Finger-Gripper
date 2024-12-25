#include "apl_main_task.h"
#include "fml_storage.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_dma.h"
#include "hal_usart.h"
#include "hal_tim.h"
#include "hal_flash.h"
#include "hal_iwdg.h"
#include "hal_fdcan.h"
#include <stdio.h>
#include <string.h>

#define DBG_TAG "boot."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

#define APP_PAGE 					(16)
uint8_t g_wait_enable = 0;
uint8_t g_upgrade_en = 0;
s_upgrade_state upgrade_state = {0};
uint8_t g_file_data[2048] = {0};

void fml_motor_reg_wirte(uint8_t index, uint16_t data, uint16_t len);
void fdcan_recv_cb(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pData);
void uart_recv_cb(uint8_t *pData, uint16_t len);
void cmd_upgrade_resp(uint8_t dev_type, uint8_t cmd, uint16_t sequence);
uint8_t sum_check(uint8_t *data, uint16_t len);
void upgrade_app_start(void);
void upgrade_motor_start(void);

void peripheral_init(void)
{
	fml_system_param_init();
	__hal_gpio_init();
	__hal_dma_init();
	__hal_fdcan1_init(fdcan_recv_cb);
	__hal_uart1_init((e_baud_type)SYSTEM_PARAM()->uart1_baud, uart_recv_cb);
	__hal_uart3_init();
	
	upgrade_state.can_id = 0x01; //关节CAN ID
	upgrade_state.dev_id = SYSTEM_PARAM()->device_id;
	g_upgrade_en = SYSTEM_PARAM()->upgrade_en;
	
	if(g_upgrade_en == 1) //升级控制板
	{
		LOG_D("upgrade ctrl\r\n");
		HAL_Delay(1200);
		fml_motor_reg_wirte(0x49, 0, 3); //电机跳转APP
		HAL_Delay(100);
		upgrade_app_start();
	}
	else if(g_upgrade_en == 2) //升级关节
	{
		LOG_D("upgrade motor\r\n");
		HAL_Delay(1000); //等待电机启动
		upgrade_motor_start();
	}
	else 
	{
		//HAL_Delay(1200); //等到电机启动
		//fml_motor_reg_wirte(0x49, 0, 3); //电机跳转APP
		//HAL_Delay(100);
		//upgrade_state.state = 1; //控制板跳转APP
		g_wait_enable = 1; //延时5秒启动，等待上位机升级指令，用于防止app损坏可以直接通过boot升级
	}
}

void upgrade_app_start(void)
{
	upgrade_state.file_size = 0;
	upgrade_state.data_num = 0;
	upgrade_state.page_num = APP_PAGE;
	cmd_upgrade_resp(g_upgrade_en, E_UPGRADE_IMG_REQ, 0);
	g_wait_enable = 1;
}

void upgrade_motor_start(void)
{
	cmd_upgrade_resp(g_upgrade_en, 0x05, 0); //由控制板发送升级请求
	g_wait_enable = 1;
}

//写寄存器
void fml_motor_reg_wirte(uint8_t index, uint16_t data, uint16_t len)
{
	uint8_t send_data[100] = {0};
	
	send_data[0] = E_CMD_WR;
	send_data[1] = index;
	send_data[2] = LO_UINT16(data);
	send_data[3] = HI_UINT16(data);
	
	hal_fdcan1_send(upgrade_state.can_id ,send_data, len);
}

//给关节发送数据
void fml_motor_frame_send(uint8_t *data, uint16_t len)
{	
	hal_fdcan1_send(upgrade_state.can_id ,data, len);
}

//响应
void cmd_data_resp(uint8_t cmd, uint8_t *pData, uint16_t len)
{
	uint8_t send_data[100] = {0};
	uint16_t send_len = 6  + len;
	
	send_data[0] = 0xEE;
	send_data[1] = 0x16;
	send_data[2] = upgrade_state.dev_id;
	send_data[3] = len + 1;
	send_data[4] = cmd;
	if(len > 0)
	{
		memcpy(&send_data[5], pData, len);
	}
	send_data[send_len - 1] = sum_check(&send_data[2], send_len-3);
	
	hal_uart1_send(send_data, send_len);
}

//响应
void cmd_upgrade_resp(uint8_t dev_type, uint8_t cmd, uint16_t sequence)
{
	uint8_t send_data[10] = {0};
	uint16_t send_len = 10;
	
	send_data[0] = 0xEE;
	send_data[1] = 0x16;
	send_data[2] = upgrade_state.dev_id;
	send_data[3] = 5;
	send_data[4] = CMD_MC_UPGRADE;
	send_data[5] = dev_type;
	send_data[6] = cmd;
	send_data[7] = LO_UINT16(sequence);
	send_data[8] = HI_UINT16(sequence);
	send_data[9] = sum_check(&send_data[2], 10-3);
	hal_uart1_send(send_data, send_len);
}

//升级控制板
void upgrade_ctrl_handel(uint8_t *pData, uint16_t len)
{
	uint8_t data_len = len - 10;
	uint8_t type = pData[5];
	uint8_t index = pData[6];
	upgrade_state.frame_sequence = BUILD_UINT16(pData[7], pData[8]);

	if(index == E_UPGRADE_IMG_REQ) //镜像包 2
	{
		g_wait_enable = 0;
		memcpy(&g_file_data[upgrade_state.data_num], &pData[9], data_len);
		upgrade_state.data_num += data_len;
		upgrade_state.file_size += data_len;
		//LOG_D("get pkg:%d %d %d \r\n", upgrade_state.data_num, data_len, upgrade_state.frame_sequence);
		if(upgrade_state.data_num >= 2048)
		{
			//LOG_D("write flah %d\r\n", upgrade_state.page_num);
			while(fml_flash_app_write(upgrade_state.page_num, (uint64_t*)g_file_data) == 0) 
			{				
				cmd_upgrade_resp(type, E_UPGRADE_FAILED, upgrade_state.frame_sequence);
				return;
			}
			upgrade_state.data_num = 0;
			upgrade_state.page_num++;
		}
		if(upgrade_state.page_num > 51)
		{
			cmd_upgrade_resp(type, E_UPGRADE_FAILED, upgrade_state.frame_sequence);
		}
		else
		{			
			cmd_upgrade_resp(type, E_UPGRADE_IMG_REQ, upgrade_state.frame_sequence);
		}
	}
	else if(index == E_UPGRADE_FINISH) //升级完成 3
	{
		if(upgrade_state.data_num > 0)
		{
			while(fml_flash_app_write(upgrade_state.page_num, (uint64_t*)g_file_data) == 0) 
			{				
				cmd_upgrade_resp(type, E_UPGRADE_FAILED, upgrade_state.frame_sequence);
				return;
			}
		}
		cmd_upgrade_resp(type, E_UPGRADE_FINISH, upgrade_state.frame_sequence);
		HAL_Delay(100);
		//upgrade_state.state = 1; //跳转APP
	}
	else if(index == E_UPGRADE_FAILED) //升级失败 4
	{
		NVIC_SystemReset(); //重启
	}
	else if(index == E_UPGRADE_STOP) //不升级 0
	{
//		upgrade_state.state = 1; //跳转APP
	}
	else if(index == E_UPGRADE_AGREE_RESTART) //升级 1 同意升级请求 6
	{
	}
	else if(index == E_UPGRADE_SART) //升级 1 同意升级请求 6
	{
		g_upgrade_en = 1;
		upgrade_app_start();
	}
}

//升级电机
void upgrade_motor_handel(uint8_t *pData, uint16_t len)
{
	uint8_t index = pData[6];
	//upgrade_state.frame_sequence = BUILD_UINT16(pData[7], pData[8]);
	
	if(index == E_UPGRADE_IMG_REQ) //镜像包 2
	{
		g_wait_enable = 0;
		fml_motor_frame_send(&pData[9], len - 10);
	}
	else if(index == E_UPGRADE_FINISH) //升级完成 3
	{
		fml_motor_reg_wirte(0x49, 0x03, 3); 
	}
	else if(index == E_UPGRADE_FAILED) //升级失败 4
	{
		fml_motor_reg_wirte(0x49, 0x04, 3);
	}
	else if(index == E_UPGRADE_AGREE_RESTART) //同意升级请求 6
	{
		LOG_D("get cmd 06\r\n");
		fml_motor_reg_wirte(0x49, 0x06, 3);
	}
	else if(index == E_UPGRADE_STOP) //不升级 0
	{
		//fml_motor_reg_wirte(0x49, 0, 3); 
	}
	else if(index == E_UPGRADE_SART) //升级 1
	{
		//fml_motor_reg_wirte(0x49, 1, 3); 
		g_upgrade_en = 2;
		upgrade_motor_start();
	}
	else if(index == E_UPGRADE_RESTART) // 5
	{
		
	}
}

//关节can数据接收回调函数
void fdcan_recv_cb(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pData)
{
	uint32_t can_id = pHeader->Identifier&0x0FFF;
	//uint32_t id = can_id - 0x100;
	//uint16_t len = FDCANLEN_TO_LEN(pHeader->DataLength);
	uint8_t frame_type = (can_id>>8)&0xFF; //寄存器类型
	
	if(frame_type == 1) //寄存器读写响应
	{
		if(pData[0] == E_CMD_WR) //写寄存器
		{
			if(pData[1] == 0x49)
			{
				if(g_upgrade_en == 2 && pData[2] != 0x05)
				{
					cmd_upgrade_resp(g_upgrade_en, pData[2], 0);
					
					if(pData[2] == 0x03 || pData[2] == 0x04) //升级完成
					{
						//LOG_D("motor upgrade ok or filed\r\n");
						//upgrade_state.state = 1; //控制板跳转APP
					}
				}
			}
		}
	}
}

//串口数据接收解析函数
void uart_recv_data_prase(uint8_t *pData, uint16_t len)
{
	uint8_t cmd = pData[4]; 

	if(cmd == CMD_MC_UPGRADE) //升级相关
	{
		uint8_t upgrade_dev = pData[5]; 
		
		if(upgrade_dev == 1)
		{
			upgrade_ctrl_handel(pData, len);
		}
		else if(upgrade_dev == 2)
		{
			upgrade_motor_handel(pData, len);
		}
	}
}

//串口数据接收回调函数
void uart_recv_cb(uint8_t *pData, uint16_t len)
{
	if(pData[0] == 0xEB && pData[1] == 0x90) 
	{
		if(pData[len-1] != sum_check(&pData[2], len-3))
		{
			return;
		}
		if(pData[2] != upgrade_state.dev_id)
		{
			return;
		} 
		else if(pData[2] == 0xFF)
		{
			uart_recv_data_prase(pData, len);
		}
		else
		{
			uart_recv_data_prase(pData, len);
		}
	}
}

//倒计时退出boot
void out_boot_wait_entry(void)
{
	static uint32_t current_time = 0;
	static uint8_t timeout = 3;
	if(HAL_GetTick() > current_time + 1000) //1ms
	{
		current_time = HAL_GetTick();
		
		if(g_wait_enable)
		{
			if(timeout < 1)
			{
				LOG_D("jump to app\r\n");
				fml_motor_reg_wirte(0x49, 0, 3); //电机跳转APP
				HAL_Delay(100);
				upgrade_state.state = 1; //控制板跳转APP
			}
			timeout --;
		}
	}
}

uint8_t sum_check(uint8_t *data, uint16_t len)
{
	uint32_t sum = 0;
	for(int i = 0; i < len; i++)
	{
		sum += data[i];
	}
	
	return sum&0x000000FF;
}

#define FLASH_APP1_ADDR		0x08008000  
typedef void (*pFunction)(void);	
//pFunction appEntry;

void _load_app(uint32_t appAddress) {
	// 获取新的栈指针（SP）地址
	uint32_t appStack = *((volatile uint32_t*) appAddress);

	// 获取新的程序复位向量地址
	pFunction appEntry = (pFunction) *(((volatile uint32_t*) appAddress) + 1);

	// 禁用中断
	__set_FAULTMASK(1);
	
	// 设置新的栈指针
	__set_MSP(appStack);

	// 复位所有中断向量到默认状态
	SCB->VTOR = appAddress;
	
	// 去初始化设备
	HAL_DeInit();
	
	// 跳转到新的程序入口地址
	appEntry();
}

void app_jump_task_entry(void)
{
	if(upgrade_state.state == 1)
	{
		_load_app(ADDR_FLASH_PAGE_16);
	}
}



