#ifndef __APL_PROTOCOL_H
#define __APL_PROTOCOL_H

#include "main.h"

//状态返回
typedef enum {
	CMD_MC_OK          			= 0x01, //正常
	CMD_MC_OUT_RANGE  						, //超限位
	CMD_MC_RUNING  								, //运行中
	CMD_MC_ERR	       			= 0x55,
}e_cmd_state_type;

typedef struct {
	uint8_t cmd;
	void (*func)(uint8 *data, uint16_t len);
}s_485_protol_handle;

typedef enum {
	E_UPGRADE_STOP 					= 0x00, /*不升级 APP-BOOT*/
	E_UPGRADE_SART 					= 0x01, /*升级 APP-BOOT*/
	E_UPGRADE_IMG_REQ 			= 0x02, /*请求升级包 BOOT-APP*/
	E_UPGRADE_FINISH				= 0x03, /*升级完成 BOOT-APP*/
	E_UPGRADE_FAILED 				= 0x04, /*升级失败 BOOT-APP*/
	E_UPGRADE_RESTART 			= 0x05, /*升级请求 BOOT-APP*/
	E_UPGRADE_AGREE_RESTART = 0x06, /*同意升级请求 APP-BOOT*/
}e_upgrade_index;

//夹爪协议
typedef enum {
	CMD_MC_PARA_SAVE 						= 0x01, //参数保存到内部闪存，掉电不丢失
	CMD_MC_PARA_DEFAULT					= 0x02, //恢复默认参数
	CMD_MC_PARA_BAUD_SET				= 0x03, //设置波特率
	CMD_MC_PARA_ID_SET 					= 0x04, //设置夹爪ID
	CMD_MC_PARA_ID_GET 					= 0x05, //读取夹爪ID
	CMD_MC_MOTOR_ID_SET 				= 0x06, //设置关节ID
	
	CMD_MC_READ_FORCE_ACK				= 0x0A, //读取夹爪力数据
	
	CMD_MC_MOVE_CATCH_XG				= 0x10, //以设置的速度和力控阈值去夹取 
	CMD_MC_MOVE_RELEASE 				= 0x11, //以设置的速度松开
	CMD_MC_SET_EG_PARA 					= 0x12, //设置夹爪开口的最大最小值
	CMD_MC_READ_EG_PARA 				= 0x13, //读取夹爪开口的最大最小值
	CMD_MC_READ_EG_STATE 				= 0x14, //--(预留)
	CMD_MC_MOVE_STOPHERE 				= 0x16, //急停
	CMD_MC_ERROR_CLR            = 0x17, //清除错误
	CMD_MC_MOVE_CATCH2_XG 			= 0x18, //以设置的速度和力控阈值持续夹取
	CMD_MC_SET_FORCE_PARA 			= 0x19, //设置力限位
	CMD_MC_GET_FORCE_PARA 			= 0x1A, //读取力限位
	CMD_MC_SET_DIO_MODE 				= 0x1B, //设置IO输出模式
	CMD_MC_SET_DIO_OUT 					= 0x1C, //设置IO输出电平
	CMD_MC_GET_DIO_STATE				= 0x1D, //读取IO输出电平
	
	CMD_MC_READ_EG_RUNSTATE 		= 0x41, //读取夹爪运行状态  
	CMD_MC_READ_SYSTEM_PARAM 		= 0x42, //读取夹爪系统参数
	CMD_MC_READ_MOTOR_STATE 		= 0x43, //读取夹爪电机状态  
	
	CMD_MC_SEEKPOS 							= 0x54, //设置夹爪开口度
	
	CMD_MC_READ_ACTPOS 					= 0xD9, //读取夹爪开口度
	CMD_MC_SET_KEY_ENABLE				= 0xDA, //设置IO控制使能
	CMD_MC_SET_ZERO   					= 0xDB, //设置零位
	CMD_MC_SERVO_MOVE   				= 0xDC, //运动校准
	
	CMD_MC_SEEKPOS_SPEED_FORCE 	= 0xE5, //设置夹爪开口度、速度、力控阈值
	
	CMD_MC_UPGRADE              = 0xFE, //升级
}e_485_cmd_type; 

void apl_init_protocol_handel(void);
void apl_485_protocol_parase(uint8_t *buf, uint16_t len);
void apl_upgrade_timeout_handle(void);

uint8_t cmd_servo_stop_init(void);
uint8_t cmd_servo_plan_init(uint16_t speed, uint16_t force, uint16_t postion, uint8_t state);

#endif /* __CTRL */
