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

//关节状态
typedef enum{
	E_STATE_NONE   = 0x00, //空闲状态
	E_STATE_RUNING 			 , //单次规划
	E_STATE_FORCE_ONCE	 , //连续规划
	E_STATE_FORCE_KEEP	 , //连续规划
	E_STATE_STOP   			 , //急停
}e_motor_state;

//返回状态
enum {
	E_STATE_STOP_MAX   			= 0x01, //最大且空闲
	E_STATE_STOP_MIN		    = 0x02, //最小且空闲
	E_STATE_STOP_NONE   		= 0x03, //停止且空闲
	E_STATE_PLAN_CLOSE   		= 0x04, //正在闭合
	E_STATE_PLAN_OPEN 			= 0x05, //正在张开
	E_STATE_PLAN_PAUSE   		= 0x06, //力控过程中遇到力暂停
	E_STATE_PLAN_STOP 			= 0x07, //规划急停
};

typedef struct {
	uint8_t run_state; //运行状态
	uint8_t bak_state; //用于返回状态
	uint8_t pre_state; //用于记录上次控制
	uint8_t cmd_state; //记录规划类型，单次规划还是次序规划
	uint8_t force_enable; //夹持力控使能
	
	uint8_t sys_error; //系统错误
	
	uint8_t enable; //电机启动状态
	uint8_t timeout; //通讯超时
	uint8_t en; //使能状态
	uint16_t error; //错误码
	uint16_t open_range; //开口度
	float range_mm; //开口度
	float current; //当前电流
	float speed; //当前速度
	float postion; //当前位置
	float voltage; //当前电压
	float temp; //当前温度
	float force; //当前力
	
	float postion_targ; //目标位置
	float speed_targ; //目标速度
	float force_targ; //目标力-电流
	float force_range; //目标力-电流
}s_motor_state;

//普通帧
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
uint8_t fml_motor_limit_check(void);

//广播帧
void fml_radio_data_parse(s_queue_can_data *pData);
void fml_radio_servo_reg_write(uint8_t index, uint16_t data, uint16_t len);
void fml_radio_servo_reg_write_any(uint8_t id, uint8_t index, uint16_t data, uint16_t len);
void fml_radio_servo_reg_read(uint8_t index, uint16_t num);
void fml_radio_servo_pos_cmd(float pos, float speed, uint8_t speed_enable);
void fml_radio_servo_state_cmd(void);
void fml_radio_servo_pos_none_cmd(float pos, float speed, uint8_t speed_enable);
void fml_radio_servo_pos_force_cmd(float pos, float speed, uint8_t speed_enable);
	
void fml_servo_reg_write(uint8_t index, uint16_t data, uint16_t len);
void fml_servo_reg_read(uint8_t index, uint16_t num);
void fml_servo_state_cmd(void);
void fml_servo_pos_cmd(float pos, float speed);

//电机数据
s_motor_state *SERVO_INFO(void);
void fml_current_filter(void);


#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
