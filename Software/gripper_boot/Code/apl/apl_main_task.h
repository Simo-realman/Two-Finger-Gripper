#ifndef __APL_MAIN_TASK_H
#define __APL_MAIN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

typedef enum{
	E_CMD_RD 			= 0x01, //读
	E_CMD_WR 			= 0x02, //写
	E_CMD_WR_NR 	= 0x03, //无应答
	E_CMD_WR_REG 	= 0x04, //异步写
	E_CMD_ACTION 	= 0x05, //执行异步写
	E_CMD_IAP		 	= 0x06, //IAP更新指令
	E_CMD_RESET 	= 0x11, //恢复出厂设置
}e_cmd_type;

//夹爪协议
typedef enum {
	CMD_MC_PARA_SAVE 						= 0x01, //参数保存到内部闪存，掉电不丢失
	CMD_MC_PARA_DEFAULT					= 0x02, //恢复默认参数
	CMD_MC_PARA_BAUD_SET				= 0x03, //设置波特率
	CMD_MC_PARA_ID_SET 					= 0x04, //设置夹爪ID
	CMD_MC_PARA_ID_GET 					= 0x05, //读取夹爪ID
	CMD_MC_MOVE_CATCH_XG				= 0x10, //以设置的速度和力控阈值去夹取 
	CMD_MC_MOVE_RELEASE 				= 0x11, //以设置的速度松开
	CMD_MC_SET_EG_PARA 					= 0x12, //设置夹爪开口的最大最小值
	CMD_MC_READ_EG_PARA 				= 0x13, //读取夹爪开口的最大最小值
	CMD_MC_READ_EG_STATE 				= 0x14, //
	CMD_MC_MOVE_STOPHERE 				= 0x16, //急停
	CMD_MC_ERROR_CLR            = 0x17, //清除错误
	CMD_MC_MOVE_CATCH2_XG 			= 0x18, //以设置的速度和力控阈值持续夹取
	CMD_MC_READ_EG_RUNSTATE 		= 0x41, //读取夹爪运行状态  
	CMD_MC_SEEKPOS 							= 0x54, //设置夹爪开口度
	CMD_MC_READ_ACTPOS 					= 0xD9, //读取夹爪开口度
	CMD_MC_SEEKPOS_SPEED_FORCE 	= 0xE5, //
	
	CMD_MC_UPGRADE              = 0xFE,               
}e_485_cmd_type; 

typedef enum {
	E_UPGRADE_STOP 					= 0x00, /*不升级 APP-BOOT*/
	E_UPGRADE_SART 					= 0x01, /*升级 APP-BOOT*/
	E_UPGRADE_IMG_REQ 			= 0x02, /*请求升级包 BOOT-APP*/
	E_UPGRADE_FINISH				= 0x03, /*升级完成 BOOT-APP*/
	E_UPGRADE_FAILED 				= 0x04, /*升级失败 BOOT-APP*/
	E_UPGRADE_RESTART 			= 0x05, /*升级请求 BOOT-APP*/
	E_UPGRADE_AGREE_RESTART = 0x06, /*同意升级请求 APP-BOOT*/
}e_upgrade_index;

typedef struct {
	uint8_t can_id;
	uint8_t dev_id;
	uint8_t state; //升级状态
	uint16_t page_num;
	uint16_t data_num;
	uint32_t frame_sequence; //序列号
	uint32_t file_size; //
}s_upgrade_state;	
	
void peripheral_init(void);		
void app_jump_task_entry(void);
void out_boot_wait_entry(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
