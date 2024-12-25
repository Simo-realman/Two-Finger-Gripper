#ifndef __APL_PROTOCOL_H
#define __APL_PROTOCOL_H

#include "main.h"

#define CMD_MC_OK           0x01
#define CMD_MC_ERR	        0x55

typedef struct {
	uint8_t cmd;
	void (*func)(uint8 *data, uint16_t len);
}s_485_protol_handle;

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

void apl_init_protocol_handel(void);
void apl_485_protocol_parase(uint8_t *buf, uint16_t len);
	
#endif /* __CTRL */
