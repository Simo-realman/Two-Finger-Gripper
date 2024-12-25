#ifndef __FML_MOTOR_H
#define __FML_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "fml_queue.h"
	
typedef enum{
	E_CMD_RD 			= 0x01, //读
	E_CMD_WR 			= 0x02, //写
	E_CMD_WR_NR 	= 0x03, //无应答
	E_CMD_WR_REG 	= 0x04, //异步写
	E_CMD_ACTION 	= 0x05, //执行异步写
	E_CMD_IAP		 	= 0x06, //IAP更新指令
	E_CMD_RESET 	= 0x11, //恢复出厂设置
}e_cmd_type;

//寄存器主索引
typedef enum{
	E_REG_MAIN_SYS 			= 0x00, //驱动器状态和参数配置
	E_REG_MAIN_CUR 			= 0x01, //当前驱动器电流、速度、位置等控制信息
	E_REG_MAIN_MOT 			= 0x02, //电机模块 及关节全球唯一ID
	E_REG_MAIN_TAG 			= 0x03, //关节目标电流、速度、位置等控制信息
	E_REG_MAIN_LIT 			= 0x04, //关节速度、加速度、位置等限制信息。
	E_REG_MAIN_SEV 			= 0x05, //关节PID配置信息
	E_REG_MAIN_BRK 			= 0x06, //关节抱闸信息
	E_REG_MAIN_ENC 			= 0x07, //编码器模块相关信息和DRV器件状态信息
	E_REG_MAIN_CAL1 		= 0x08, //参数校准1
	E_REG_MAIN_CAL2 		= 0x09, //参数校准2
}e_reg_main;

//错误码
typedef enum{
	E_ERR_FOC 					= 0x0001, //FOC错误
	E_ERR_OVER_VOL 			= 0x0002, //过压
	E_ERR_UNDER_VOL			= 0x0004, //欠压
	E_ERR_VOER_TMP 			= 0x0008, //过温
	E_ERR_START 				= 0x0010, //启动错误
	E_ERR_ENCODER 			= 0x0020, //编码器错误
	E_ERR_OVER_CUR 			= 0x0040, //过流
	E_ERR_SOFT 					= 0x0080, //软件错误
	E_ERR_TMP_SENSOR 		= 0x0100, //温度传感器错误
	E_ERR_OVER_POS 			= 0x0200, //目标位置超限
	E_ERR_DRV8320 			= 0x0400, //DRV8320错误
	E_ERR_POS_TRACK 		= 0x0800, //位置跟踪误差
	E_ERR_CUR_CHECK 		= 0x1000, //电流检测错误
	E_ERR_CHECK_SELF 		= 0x2000, //自检错误
	E_ERR_CMD_POS 			= 0x4000, //位置指令超限
	E_ERR_DUOQUAN_LOSE 	= 0x8000, //多圈丢数
}e_motor_error_code;

//错误码
typedef enum{
	E_STATE_STOP   = 0x00,
	E_STATE_RUNING = 0x01,
	
}e_motor_state;
//关节状态
typedef struct {
//	float current; //当前电流
//	float speed; //当前速度
//	float postion; //当前位置
//	float voltage; //当前电压
//	float temp; //当前温度
	
	uint8_t en; //使能状态
	uint8_t run_state; //运行状态
	uint16_t error; //错误码
	
	int pos_real; //实际位置
	int current_real; //实际电流
	int voltage_real; //实际电压
	int speed_real; //实际速度
	int temp_real; //实际温度
	int force_real; //实际力矩
	
//	int pos_tag; //目标位置
//	uint16_t force_tag; //目标力矩
//	uint16_t speed_tag; //目标速度

}s_motor_state;

void fml_fdcan_data_parse(s_queue_can_data *pData);
void fml_motor_send(s_queue_can_data *pData);

void fml_motor_limit_cmd(uint8_t index, uint16_t data);
void fml_motor_frame_send(uint8_t *data, uint16_t len);

void fml_motor_reg_read(uint8_t index, uint16_t num);
void fml_motor_reg_wirte(uint8_t index, uint16_t data, uint16_t len);

void fml_motor_servo_state_cmd(void);
void fml_motor_servo_current_cmd(float current);
void fml_motor_servo_speed_cmd(float speed);	
void fml_motor_servo_pos_cmd(float pos);

void fml_motor_limit_check(void);

s_motor_state *SERVO_INFO(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
