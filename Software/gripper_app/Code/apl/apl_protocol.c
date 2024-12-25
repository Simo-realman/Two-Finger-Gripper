#include "apl_protocol.h"
#include "fml_motor.h"
#include "fml_modbus.h"
#include "fml_storage.h"
#include "fml_queue.h"
#include "fml_invoke_algo.h"
#include "hal_usart.h"
#include "hal_gpio.h"
#include "common.h"
#include "gripper_algo.h"

#include <stdio.h>
#include <string.h>

#define DBG_TAG "pro."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

s_mb_rtu_cb mbCb;
uint8_t g_upgrade_enable = 0;
static uint8_t DEV_ADDR = 0x01;
static uint8_t cmd_id = 0; //请求ID

extern s_io_state g_dio_state[2]; //DIO状态
extern uint16_t g_app_version;
extern TRANS_QUEUE qUart1Recv;
extern TRANS_QUEUE qUart1Send;
extern unsigned char g_mb_address; //modbus地址
extern s_motor_state motor_state; //电机状态
extern system_parameters_t spt;

void cmd_para_save_handle(uint8_t *buf, uint16_t len);
void cmd_para_default_handle(uint8_t *buf, uint16_t len);
void cmd_dev_id_set_handle(uint8_t *buf, uint16_t len);
void cmd_dev_id_get_handle(uint8_t *buf, uint16_t len);
void cmd_dev_baud_set_handle(uint8_t *buf, uint16_t len);
void cmd_move_catch_xg_handle(uint8_t *buf, uint16_t len);
void cmd_move_release_handle(uint8_t *buf, uint16_t len);
void cmd_set_eg_para_handle(uint8_t *buf, uint16_t len);
void cmd_read_eg_para_handle(uint8_t *buf, uint16_t len);
void cmd_move_stop_handle(uint8_t *buf, uint16_t len);
void cmd_error_clear_handle(uint8_t *buf, uint16_t len);
void cmd_move_catch2_xg_handle(uint8_t *buf, uint16_t len);
void cmd_read_eg_runstate_handle(uint8_t *buf, uint16_t len);
void cmd_set_seekpos_handle(uint8_t *buf, uint16_t len);
void cmd_read_actpos_handle(uint8_t *buf, uint16_t len);
void cmd_seekpos_speed_force_handle(uint8_t *buf, uint16_t len);
void cmd_read_system_param_handle(uint8_t *buf, uint16_t len);
void cmd_read_motor_state_handle(uint8_t *buf, uint16_t len);
void cmd_upgrade_handle(uint8_t *buf, uint16_t len);
void cmd_set_zero_handle(uint8_t *buf, uint16_t len);
void cmd_one_step_move_handle(uint8_t *buf, uint16_t len);
void cmd_set_force_para_handle(uint8_t *buf, uint16_t len);
void cmd_get_force_para_handle(uint8_t *buf, uint16_t len);
void cmd_set_io_mode_handle(uint8_t *buf, uint16_t len);
void cmd_set_io_out_handle(uint8_t *buf, uint16_t len);
void cmd_get_io_state_handle(uint8_t *buf, uint16_t len);
void cmd_get_force_handle(uint8_t *buf, uint16_t len);
void cmd_motor_id_set_handle(uint8_t *buf, uint16_t len);
void cmd_set_key_enable_handle(uint8_t *buf, uint16_t len);

#define MAX_485_CMD_NUM 29
s_485_protol_handle protol_485_handle[MAX_485_CMD_NUM] = {
	{CMD_MC_SEEKPOS_SPEED_FORCE, cmd_seekpos_speed_force_handle},
	{CMD_MC_READ_FORCE_ACK, cmd_get_force_handle},
	{CMD_MC_READ_MOTOR_STATE, cmd_read_motor_state_handle},
	{CMD_MC_PARA_SAVE, cmd_para_save_handle},
	{CMD_MC_PARA_DEFAULT, cmd_para_default_handle},
	{CMD_MC_PARA_ID_SET, cmd_dev_id_set_handle},
	{CMD_MC_PARA_ID_GET, cmd_dev_id_get_handle},
	{CMD_MC_MOTOR_ID_SET, cmd_motor_id_set_handle},
	{CMD_MC_PARA_BAUD_SET, cmd_dev_baud_set_handle},
	{CMD_MC_MOVE_CATCH_XG, cmd_move_catch_xg_handle},
	{CMD_MC_MOVE_RELEASE, cmd_move_release_handle},
	{CMD_MC_SET_EG_PARA, cmd_set_eg_para_handle},
	{CMD_MC_READ_EG_PARA, cmd_read_eg_para_handle},
	{CMD_MC_MOVE_STOPHERE, cmd_move_stop_handle},
	{CMD_MC_ERROR_CLR, cmd_error_clear_handle},
	{CMD_MC_MOVE_CATCH2_XG, cmd_move_catch2_xg_handle},
	{CMD_MC_READ_EG_RUNSTATE, cmd_read_eg_runstate_handle},
	{CMD_MC_SEEKPOS, cmd_set_seekpos_handle},
	{CMD_MC_READ_ACTPOS, cmd_read_actpos_handle},
	{CMD_MC_READ_SYSTEM_PARAM, cmd_read_system_param_handle},
	{CMD_MC_UPGRADE, cmd_upgrade_handle},
	{CMD_MC_SET_ZERO, cmd_set_zero_handle},
	{CMD_MC_SERVO_MOVE, cmd_one_step_move_handle},
	{CMD_MC_SET_FORCE_PARA, cmd_set_force_para_handle},
	{CMD_MC_GET_FORCE_PARA, cmd_get_force_para_handle},
	{CMD_MC_SET_DIO_MODE, cmd_set_io_mode_handle},
	{CMD_MC_SET_DIO_OUT, cmd_set_io_out_handle},
	{CMD_MC_GET_DIO_STATE, cmd_get_io_state_handle},
	{CMD_MC_SET_KEY_ENABLE, cmd_set_key_enable_handle},
};

static uint8_t sum_check(uint8_t *data, uint16_t len)
{
	uint32_t sum = 0;
	for(int i = 0; i < len; i++)
	{
		sum += data[i];
	}
	
	return sum&0x000000FF;
}

void apl_485_protocol_parase(uint8_t *buf, uint16_t len)
{
	uint8_t i = 0;
	if(buf[len-1] != sum_check(&buf[2], len-3))
	{
		return;
	}
	cmd_id = buf[2];
	
	if(buf[2] == 0xFF)
	{
		for(i = 0; i < MAX_485_CMD_NUM; i++)
		{
			if(protol_485_handle[i].cmd == buf[4])
			{
				protol_485_handle[i].func(buf, len);
			}
		}
		return;
	}
	if(buf[2] == DEV_ADDR)
	{
		for(i = 0; i < MAX_485_CMD_NUM; i++)
		{
			if(protol_485_handle[i].cmd == buf[4])
			{
				protol_485_handle[i].func(buf, len);
			}
		}
	}
}

//数据响应
void cmd_data_resp(uint8_t cmd, uint8_t *pData, uint16_t len)
{
	s_queue_uart_data send_data = {0};
	
	send_data.len = 6  + len;
	
	send_data.data[0] = 0xEE;
	send_data.data[1] = 0x16;
	send_data.data[2] = DEV_ADDR;
	send_data.data[3] = len + 1;
	send_data.data[4] = cmd;
	memcpy(&send_data.data[5], pData, len);
	send_data.data[send_data.len - 1] = sum_check(&send_data.data[2], send_data.len-3);
	if(!FullQueue(&qUart1Send) && cmd_id != 0xFF)
	{
		Enqueue(&qUart1Send, &send_data);
	}
}

//状态响应
void cmd_state_resp(uint8_t cmd, uint8_t state)
{
	s_queue_uart_data send_data = {0};
	
	send_data.len = 7;
	
	send_data.data[0] = 0xEE;
	send_data.data[1] = 0x16;
	send_data.data[2] = DEV_ADDR;
	send_data.data[3] = 2;
	send_data.data[4] = cmd;
	send_data.data[5] = state;
	send_data.data[6] = sum_check(&send_data.data[2], 4);
	if(!FullQueue(&qUart1Send) && cmd_id != 0xFF)
	{
		Enqueue(&qUart1Send, &send_data);
	}
}

//升级响应
void cmd_upgrade_resp(uint8_t dev_type, uint8_t cmd, uint16_t sequence)
{
	uint8_t send_data[10] = {0};
	uint16_t send_len = 10;
	
	send_data[0] = 0xEE;
	send_data[1] = 0x16;
	send_data[2] = DEV_ADDR;
	send_data[3] = 5;
	send_data[4] = CMD_MC_UPGRADE;
	send_data[5] = dev_type;
	send_data[6] = cmd;
	send_data[7] = LO_UINT16(sequence);
	send_data[8] = HI_UINT16(sequence);
	send_data[9] = sum_check(&send_data[2], 10-3);
	hal_uart1_send(send_data, send_len);
}

//运动规划初始化
uint8_t cmd_servo_plan_init(uint16_t _speed, uint16_t _force, uint16_t _postion, uint8_t _state)
{
	uint16_t speed = _speed;
	uint16_t force = _force;
	uint16_t postion = _postion;
	
	// || motor_state.current > 4000 || motor_state.current < -4000
	if(motor_state.error != 0 || motor_state.sys_error != 0)
	{
		return RM_ERROR;
	}
	
	//检测阈值
	if(speed > RM_SPEED_MAX) speed = RM_SPEED_MAX;
	if(speed < RM_SPEED_MIN) speed = RM_SPEED_MIN;
	if(force > RM_FORCE_MAX) force = RM_FORCE_MAX;
	if(force < RM_FORCE_MIN) force = RM_FORCE_MIN;
	if(postion > RM_POS_MAX) postion = RM_POS_MAX;
	if(postion < RM_POS_MIN) postion = RM_POS_MIN;
	//检测限位
	if(postion > spt.pos_limit_max)
	{
		postion = spt.pos_limit_max;
	}
	else if(postion < spt.pos_limit_min)
	{
		postion = spt.pos_limit_min;
	}	
	//力限位
	motor_state.force_targ = force + RM_CURRENT_OFFSET; //800用于克服静摩擦
	
	if(_state == E_STATE_PLAN_OPEN) //松开指令  
	{
		motor_state.force_targ = RM_CURRENT_MAX; //开启动作不做任何力限制
	} 
	else if(_state == E_STATE_PLAN_CLOSE) //夹持指令
	{
		if(motor_state.force_enable == RM_TRUE) //如果当前有物体夹持则退出规划
		{
			return RM_OK;
		}
	}
	motor_state.force_enable = RM_FALSE; //停止力控
	
	//状态位
	motor_state.force = 0; //清空实时力
	SYSTEM_PARAM()->servo_force = force; //记录力值
	motor_state.speed_targ = speed/1000.0; //速度百分比
	motor_state.postion_targ = RM_RANGE_MAX*((float)postion/1000.0); //1000<->65目标位置
	motor_state.bak_state = _state; //返回状态
	motor_state.pre_state = _state; //上次状态
	motor_state.run_state = E_STATE_RUNING; //开始规划
	
	//LOG_D("p1:%.2f p2:%.2f p3:%.2f\r\n", motor_state.postion, motor_state.postion_targ, gripper_ikine(0, motor_state.postion_targ));
	return RM_OK;
}

//急停规划初始化
uint8_t cmd_servo_stop_init(void)
{
	motor_state.speed_targ = 1;
	motor_state.postion_targ = 0; 
	run_emergency();
	SERVO_INFO()->run_state = E_STATE_STOP;
	return RM_OK;
}

//设置IO输出模式
void cmd_set_io_mode_handle(uint8_t *buf, uint16_t len)
{
	uint8_t num = buf[5];
	uint8_t mode = buf[6];
	
	if(mode == 0) //输入模式
	{
		if(num == 0) //IO0
		{
			DOUT1(0);
			g_dio_state[0].pin_mode = E_DIO_MODE_IN;
		}
		else if(num == 1) //IO1
		{
			DOUT2(0);
			g_dio_state[1].pin_mode = E_DIO_MODE_IN;
		}
	}
	else if(mode == 1) //输出模式-默认输出低电平
	{
		if(num == 0) //IO0
		{
			DOUT1(1); //低电平
			g_dio_state[0].pin_mode = E_DIO_MODE_OUT;
		}
		else if(num == 1) //IO1
		{
			DOUT2(1);//低电平
			g_dio_state[1].pin_mode = E_DIO_MODE_OUT;
		}
	}
	cmd_state_resp(CMD_MC_SET_DIO_MODE, CMD_MC_OK);
}

//设置IO输出电平
void cmd_set_io_out_handle(uint8_t *buf, uint16_t len)
{
	uint8_t num = buf[5];
	uint8_t out = buf[6];
	
	if(num == 0) //IO1
	{
		if(g_dio_state[0].pin_mode != E_DIO_MODE_OUT)
		{
			cmd_state_resp(CMD_MC_SET_DIO_OUT, CMD_MC_ERR);
			return;
		}
		if(out == 0) //输出低电平
		{
			DOUT1(1); //IO输出和对外输出特性相反
		}
		else if(out == 1) //输出高电平
		{
			DOUT1(0);
		}
	}
	else if(num == 1) //IO2
	{
		if(g_dio_state[1].pin_mode != E_DIO_MODE_OUT)
		{
			cmd_state_resp(CMD_MC_SET_DIO_OUT, CMD_MC_ERR);
			return;
		}
		
		if(out == 0) //输出低电平
		{
			DOUT2(1);
		}
		else if(out == 1) //输出低电平
		{
			DOUT2(0);
		}
	}
	cmd_state_resp(CMD_MC_SET_DIO_OUT, CMD_MC_OK);
}

//设置按键控制使能
void cmd_set_key_enable_handle(uint8_t *buf, uint16_t len)
{
	if(buf[5] == 0) 
	{
		SYSTEM_PARAM()->key_enable = 0;
	}
	else
	{
		//设置输入模式
		DOUT1(0);
		g_dio_state[0].pin_mode = E_DIO_MODE_IN;
		DOUT2(0);
		g_dio_state[1].pin_mode = E_DIO_MODE_IN;
		SYSTEM_PARAM()->key_enable = 1;
	}
	cmd_state_resp(CMD_MC_SET_KEY_ENABLE, CMD_MC_OK);
}

//读取IO状态
void cmd_get_io_state_handle(uint8_t *buf, uint16_t len)
{	
	uint8_t data[4] = {0};
	
	data[0] = g_dio_state[0].pin_mode;
	data[1] = DIN1;
	data[2] = g_dio_state[1].pin_mode;
	data[3] = DIN2;
	cmd_data_resp(CMD_MC_GET_DIO_STATE, data, 4);
}

//关节设置零位
void cmd_set_zero_handle(uint8_t *buf, uint16_t len)
{
	fml_radio_servo_reg_write(0x0E, 1, 4); //设置当前为零位
	HAL_Delay(100);
	fml_radio_servo_reg_write(0x0C, 1, 4); //保存到flash
	HAL_Delay(100);
	fml_radio_servo_reg_write(0x0A, 1, 4); //上使能
	cmd_state_resp(CMD_MC_SET_ZERO, CMD_MC_OK);
}

//关节单步运动
void cmd_one_step_move_handle(uint8_t *buf, uint16_t len)
{
	float pos = SERVO_INFO()->postion;
	float step = 0; //角度
	memcpy(&step, &buf[5], sizeof(step));
	pos += step;
	fml_radio_servo_pos_none_cmd(pos, 0, 0);  //单步运动
	cmd_state_resp(CMD_MC_SERVO_MOVE, CMD_MC_OK);
}

//升级指令
void cmd_upgrade_handle(uint8_t *buf, uint16_t len)
{
	uint8_t type = buf[5]; //升级设备
	uint8_t index = buf[6]; //命令号
	uint16_t sequence = BUILD_UINT16(buf[7], buf[8]);

	if(index == E_UPGRADE_SART) //开始升级
	{
		SYSTEM_PARAM()->upgrade_en = type;
		if(RM_OK != fml_storage_save(E_DATA_ALL)) //保存flash
		{		
			cmd_upgrade_resp(type, E_UPGRADE_FAILED, sequence);
		}
		else
		{
			g_upgrade_enable = 20; //倒计时等待10s
		}
	}
}

//升级等待超时处理
void apl_upgrade_timeout_handle(void)
{
	g_upgrade_enable = 0; 
	cmd_upgrade_resp(SYSTEM_PARAM()->upgrade_en, E_UPGRADE_FAILED, 0);
	SYSTEM_PARAM()->upgrade_en = 0;
	if(RM_OK != fml_storage_save(E_DATA_ALL)) //保存flash
	{		
		
	}
}

//参数保存到闪存
void cmd_para_save_handle(uint8_t *buf, uint16_t len)
{
	if(RM_OK == fml_storage_save(E_DATA_ALL))
	{		
		cmd_state_resp(CMD_MC_PARA_SAVE, CMD_MC_OK);
	}
	else 
	{
		cmd_state_resp(CMD_MC_PARA_SAVE, CMD_MC_ERR);
	}
}

//恢复到默认参数
void cmd_para_default_handle(uint8_t *buf, uint16_t len)
{
	fml_system_param_default();
	cmd_state_resp(CMD_MC_PARA_DEFAULT, CMD_MC_OK);
}

//设置设备ID
void cmd_dev_id_set_handle(uint8_t *buf, uint16_t len)
{
	if(buf[5] > 0 && buf[5] < 255) 
	{
		SYSTEM_PARAM()->device_id = buf[5];
		//DEV_ADDR = buf[5]; //485设备ID
		//g_mb_address = buf[5]; //RTU设备ID
		cmd_state_resp(CMD_MC_PARA_ID_SET, CMD_MC_OK);
	}
	else
	{
		cmd_state_resp(CMD_MC_PARA_ID_SET, CMD_MC_ERR);
	}
	LOG_D("set id:%d\r\n", SYSTEM_PARAM()->device_id);
}

//设置关节ID
void cmd_motor_id_set_handle(uint8_t *buf, uint16_t len)
{
	if(buf[5] > 0 && buf[5] < 16) 
	{
		SYSTEM_PARAM()->motor_id = buf[5];

		for(uint8_t id = 1; id < 15; id++) //遍历所有关节ID设置ID
		{
			HAL_Delay(100);
			fml_radio_servo_reg_write_any(id, 0x01, buf[5], 4); //关节上使能
			HAL_Delay(100);
			fml_radio_servo_reg_write_any(id, 0x0C, 1, 4); //保存到flash
		}
		cmd_state_resp(CMD_MC_MOTOR_ID_SET, CMD_MC_OK);
	}
	else
	{
		cmd_state_resp(CMD_MC_MOTOR_ID_SET, CMD_MC_ERR);
	}
}

//获取设备ID
void cmd_dev_id_get_handle(uint8_t *buf, uint16_t len)
{
	s_queue_uart_data send_data = {0};
	send_data.len = 7;
	send_data.data[0] = 0xEE;
	send_data.data[1] = 0x16;
	send_data.data[2] = DEV_ADDR;
	send_data.data[3] = 2;
	send_data.data[4] = CMD_MC_PARA_ID_GET;
	send_data.data[5] = DEV_ADDR;
	send_data.data[6] = sum_check(&send_data.data[2], send_data.len-3);
	if(!FullQueue(&qUart1Send))
	{
		Enqueue(&qUart1Send, &send_data);
	}
}

//设置波特率
void cmd_dev_baud_set_handle(uint8_t *buf, uint16_t len)
{
	if(buf[5] >= 0 && buf[5] < 6) 
	{
		SYSTEM_PARAM()->uart1_baud = buf[5];
		cmd_state_resp(CMD_MC_PARA_BAUD_SET, CMD_MC_OK);
	}
	else
	{
		cmd_state_resp(CMD_MC_PARA_BAUD_SET, CMD_MC_ERR);
	}
	LOG_D("baud:%d\r\n", SYSTEM_PARAM()->uart1_baud);
}

//以设置的速度和力控阈值去夹取 
void cmd_move_catch_xg_handle(uint8_t *buf, uint16_t len)
{
	uint16_t speed = BUILD_UINT16(buf[5], buf[6]);
	uint16_t force = BUILD_UINT16(buf[7], buf[8]);
	uint16_t postion = SYSTEM_PARAM()->pos_limit_min;
	
	motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
	cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_CLOSE);
	cmd_state_resp(CMD_MC_MOVE_CATCH_XG, CMD_MC_OK);
}

//以设置的速度和力控阈值持续夹取
void cmd_move_catch2_xg_handle(uint8_t *buf, uint16_t len)
{
	uint16_t speed = BUILD_UINT16(buf[5], buf[6]);
	uint16_t force = BUILD_UINT16(buf[7], buf[8]);
	uint16_t postion = SYSTEM_PARAM()->pos_limit_min;
	
	motor_state.cmd_state = E_STATE_FORCE_KEEP; //持续夹取
	cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_CLOSE);
	cmd_state_resp(CMD_MC_MOVE_CATCH2_XG, CMD_MC_OK);
}

//以设置的速度松开
void cmd_move_release_handle(uint8_t *buf, uint16_t len)
{
	uint16_t speed = BUILD_UINT16(buf[5], buf[6]);
	uint16_t force = SYSTEM_PARAM()->servo_force;
	uint16_t postion = SYSTEM_PARAM()->pos_limit_max;
	
	cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_OPEN);
	cmd_state_resp(CMD_MC_MOVE_RELEASE, CMD_MC_OK);
}

//急停
void cmd_move_stop_handle(uint8_t *buf, uint16_t len)
{	
	cmd_servo_stop_init();
	cmd_state_resp(CMD_MC_MOVE_STOPHERE, CMD_MC_OK);
}

//清除错误
void cmd_error_clear_handle(uint8_t *buf, uint16_t len)
{
	if(motor_state.sys_error&(0x01<<4))
	{
		fml_motor_reg_wirte(0x49, 0, 3); //电机跳转APP
	}
	fml_motor_reg_wirte(0x0F, 0x01, 3); //清除关节错误
	cmd_state_resp(CMD_MC_ERROR_CLR, CMD_MC_OK);
}

//设置夹爪开口的最大最小值
void cmd_set_eg_para_handle(uint8_t *buf, uint16_t len)
{
	uint16_t max = BUILD_UINT16(buf[5], buf[6]);
	uint16_t min = BUILD_UINT16(buf[7], buf[8]);
	if(max > RM_POS_MAX || max < RM_POS_MIN || min > RM_POS_MAX || min < RM_POS_MIN || max <= min)
	{
		cmd_state_resp(CMD_MC_SET_EG_PARA, CMD_MC_ERR);
	}
	else
	{
		// 如果不在当前限位之内，移动到限位。		
		if(motor_state.open_range > max)
		{
			cmd_servo_plan_init(SYSTEM_PARAM()->servo_speed, SYSTEM_PARAM()->servo_force, max, E_STATE_PLAN_CLOSE);
		}
		else if(motor_state.open_range < min)
		{
			cmd_servo_plan_init(SYSTEM_PARAM()->servo_speed, SYSTEM_PARAM()->servo_force, min, E_STATE_PLAN_OPEN);
		}
		SYSTEM_PARAM()->pos_limit_max = max;
		SYSTEM_PARAM()->pos_limit_min = min;
		cmd_state_resp(CMD_MC_SET_EG_PARA, CMD_MC_OK);
	}
}

//读取夹爪开口的最大最小值
void cmd_read_eg_para_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[4] = {0};
	
	data[0] = LO_UINT16(SYSTEM_PARAM()->pos_limit_max);
	data[1] = HI_UINT16(SYSTEM_PARAM()->pos_limit_max);
	data[2] = LO_UINT16(SYSTEM_PARAM()->pos_limit_min);
	data[3] = HI_UINT16(SYSTEM_PARAM()->pos_limit_min);
	
	cmd_data_resp(CMD_MC_READ_EG_PARA, data, 4);
}

//设置力
void cmd_set_force_para_handle(uint8_t *buf, uint16_t len)
{
	SYSTEM_PARAM()->servo_force = BUILD_UINT16(buf[5], buf[6]);
	cmd_state_resp(CMD_MC_SET_FORCE_PARA, CMD_MC_OK);
}

//读取力
void cmd_get_force_para_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[2] = {0};
	
	data[0] = LO_UINT16(SYSTEM_PARAM()->servo_force);
	data[1] = HI_UINT16(SYSTEM_PARAM()->servo_force);
	
	cmd_data_resp(CMD_MC_GET_FORCE_PARA, data, 2);
}

//指定夹爪开口度
void cmd_set_seekpos_handle(uint8_t *buf, uint16_t len)
{
	uint16_t speed = SYSTEM_PARAM()->servo_speed;
	uint16_t force = SYSTEM_PARAM()->servo_force;
	uint16_t postion = BUILD_UINT16(buf[5], buf[6]);
	
	uint8_t state = E_STATE_PLAN_OPEN;
	if(postion < SERVO_INFO()->open_range)
	{
		state = E_STATE_PLAN_CLOSE;
		motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
	}
	cmd_servo_plan_init(speed, force, postion, state);
	cmd_state_resp(CMD_MC_SEEKPOS, CMD_MC_OK);
}

//读取夹爪开口度
void cmd_read_actpos_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[2] = {0};
	uint16_t pos = SERVO_INFO()->open_range;
	data[0] = LO_UINT16(pos);
	data[1] = HI_UINT16(pos);
	cmd_data_resp(CMD_MC_READ_ACTPOS, data, 2);
}

//读取夹爪运行状态-兼容因时协议
void cmd_read_eg_runstate_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[7] = {0};
	s_motor_state *state = SERVO_INFO();
	
	//状态获取
	uint8_t bakstate = state->bak_state;
	if(state->bak_state == E_STATE_STOP_NONE) 
	{
		if(state->open_range == RM_POS_MAX)
		{
			bakstate = E_STATE_STOP_MAX;
		}
		else if(state->open_range == RM_POS_MIN)
		{
			bakstate = E_STATE_STOP_MIN;
		}
	}	
	data[0] = bakstate;
	data[1] = state->sys_error;
	data[2] = (uint8_t)state->temp;
	data[3] = LO_UINT16(state->open_range);
	data[4] = HI_UINT16(state->open_range); //开口度
#ifdef RM_HIGHT_FORCE_ENABLE			
	data[5] = LO_UINT16((uint16_t)(state->force/2));
	data[6] = HI_UINT16((uint16_t)(state->force/2)); //加持力
#else 
	data[5] = LO_UINT16((uint16_t)(state->force));
	data[6] = HI_UINT16((uint16_t)(state->force)); //加持力	
#endif
	cmd_data_resp(CMD_MC_READ_EG_RUNSTATE, data, 7);
}

//读取夹爪力数据
void cmd_get_force_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[4] = {0};
	s_motor_state *state = SERVO_INFO();
	data[2] = LO_UINT16((uint16_t)(state->force));
	data[3] = HI_UINT16((uint16_t)(state->force)); //加持力
	cmd_data_resp(CMD_MC_READ_FORCE_ACK, data, 4);
}

//设置夹爪开口度、速度、力控阈值-用于透传-不响应返回
void cmd_seekpos_speed_force_handle(uint8_t *buf, uint16_t len)
{
	uint16_t postion = BUILD_UINT16(buf[5], buf[6]);
	uint16_t speed = BUILD_UINT16(buf[7], buf[8]);
	uint16_t force = BUILD_UINT16(buf[9], buf[10]);	
	uint8_t state = E_STATE_PLAN_CLOSE;
	if(postion > SERVO_INFO()->open_range)
	{
		state = E_STATE_PLAN_OPEN;
	}
	else
	{
		motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
	}
	cmd_servo_plan_init(speed, force, postion, state);
}

//读取系统参数
void cmd_read_system_param_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[12] = {0};
	
	system_parameters_t *param = SYSTEM_PARAM();
	
	data[0] = param->device_id;
	data[1] = param->uart1_baud;
	
	data[2] = LO_UINT16(param->pos_limit_min);
	data[3] = HI_UINT16(param->pos_limit_min);
	data[4] = LO_UINT16(param->pos_limit_max);
	data[5] = HI_UINT16(param->pos_limit_max);
	data[6] = LO_UINT16(param->servo_speed);
	data[7] = HI_UINT16(param->servo_speed);
	
	data[8] = LO_UINT16(param->servo_force);
	data[9] = HI_UINT16(param->servo_force);
	data[10] = LO_UINT16(g_app_version);
	data[11] = HI_UINT16(g_app_version);
	
	cmd_data_resp(CMD_MC_READ_SYSTEM_PARAM, data, 14);
}

//读取电机状态
void cmd_read_motor_state_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[31] = {0};
	float force_tmp = 0;
	s_motor_state *state = SERVO_INFO();
	
	//状态获取
	uint8_t bakstate = state->bak_state;
	if(state->bak_state == E_STATE_STOP_NONE) 
	{
		if(state->open_range == RM_POS_MAX)
		{
			bakstate = E_STATE_STOP_MAX;
		}
		else if(state->open_range == RM_POS_MIN)
		{
			bakstate = E_STATE_STOP_MIN;
		}
	}
	
	data[0] = state->en;
	data[1] = bakstate;
	data[2] = LO_UINT16(state->error);
	data[3] = HI_UINT16(state->error);
	memcpy(&data[4], &state->current, sizeof(state->current));
	memcpy(&data[8], &state->voltage, sizeof(state->voltage));
	memcpy(&data[12], &state->postion, sizeof(state->postion));
	memcpy(&data[16], &state->speed, sizeof(state->speed));
	memcpy(&data[20], &state->temp, sizeof(state->temp));
#ifdef RM_HIGHT_FORCE_ENABLE	
	force_tmp = state->force/2.0;	
	memcpy(&data[24], &force_tmp, sizeof(force_tmp)); //加持力
#else 
	memcpy(&data[24], &state->force, sizeof(state->force)); //加持力	
#endif	
	data[28] = LO_UINT16(state->open_range);
	data[29] = HI_UINT16(state->open_range);
	data[30] = state->sys_error; //系统错误
	
	cmd_data_resp(CMD_MC_READ_MOTOR_STATE, data, 31);
}

//写线圈处理
uint8_t mb_wirte_coils_cb(uint8_t *buf, uint16_t len)
{
	uint8_t func = buf[1];
	uint16_t addr = (buf[2] << 8) | buf[3];
  uint16_t value = 0;
	
	if(addr == MB_COILS_DIO_MODE_0) /*DIO0模式设置 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_COIL && (value == 0 || value == 0xFF00))
		{
			if(value == 0) //输入模式
			{
				DOUT1(0);
				g_dio_state[0].pin_mode = E_DIO_MODE_IN;
			}
			else //输出模式
			{
				DOUT1(1); //默认输出低电平
				g_dio_state[0].pin_mode = E_DIO_MODE_OUT;
			}
			return RM_OK;
		}
	} 
	else if(addr == MB_COILS_DIO_STATE_0) /*DIO0状态 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_COIL && (value == 0 || value == 0xFF00))
		{
			if(g_dio_state[0].pin_mode != E_DIO_MODE_OUT)
			{
				return RM_ERROR;
			}
			if(value == 0) 
			{
				DOUT1(1); //输出低电平
			}
			else 
			{
				DOUT1(0); //输出高电平
			}
			return RM_OK;
		}
	}
	else if(addr == MB_COILS_DIO_MODE_1) /*DIO1模式设置 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_COIL && (value == 0 || value == 0xFF00))
		{
			if(value == 0) //输入模式
			{
				DOUT2(0);
				g_dio_state[1].pin_mode = E_DIO_MODE_IN;
			}
			else //输出模式
			{
				DOUT2(1); //默认输出低电平
				g_dio_state[1].pin_mode = E_DIO_MODE_OUT;
			}
			return RM_OK;
		}
	} 
	else if(addr == MB_COILS_DIO_STATE_1) /*DIO0状态 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_COIL && (value == 0 || value == 0xFF00))
		{
			if(g_dio_state[1].pin_mode != E_DIO_MODE_OUT)
			{
				return RM_ERROR;
			}
			if(value == 0) 
			{
				DOUT2(1); //输出低电平
			}
			else 
			{
				DOUT2(0); //输出高电平
			}
			return RM_OK;
		}
	}
}

//写保持寄存器处理
uint8_t mb_wirte_holding_cb(uint8_t *buf, uint16_t len)
{
	uint8_t func = buf[1];
	uint16_t addr = (buf[2] << 8) | buf[3];
  uint16_t value = 0;
	
	if(addr == MB_REG_PARAM_SAVE) /*保存参数 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING && (value == 0 || value == 1))
		{
			if(value == 1)
			{
				return fml_storage_save(E_DATA_ALL);
			}
			return RM_OK;
		}
	}
	else if(addr == MB_REG_PARAM_DEFAULT) /*参数恢复默认 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING && (value == 0 || value == 1))
		{
			if(value == 1)
			{
				fml_system_param_default();
			}
			return RM_OK;
		}
	}
	else if(addr == MB_REG_DEV_ID) /*设置设备ID (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= 0 && value < 255) 
			{
				SYSTEM_PARAM()->device_id = value;
//				DEV_ADDR = value; //485设备ID
//				g_mb_address = value; //RTU设备ID
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_UART_BAUD) /*设置串口波特率 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= 0 && value < 6) 
			{
				SYSTEM_PARAM()->uart1_baud = value;
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_CATCH) /*力控夹取 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			uint16_t speed = SYSTEM_PARAM()->servo_speed;
			uint16_t force = SYSTEM_PARAM()->servo_force;
			uint16_t postion = SYSTEM_PARAM()->pos_limit_min;
			
			if(value == 0)
			{
				motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
				cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_CLOSE);
				return RM_OK;
			}
			else if(value == 1)
			{
				motor_state.cmd_state = E_STATE_FORCE_KEEP; //持续夹取
				cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_CLOSE);
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_LOOSE) /*松开 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			uint16_t speed = SYSTEM_PARAM()->servo_speed;
			uint16_t force = SYSTEM_PARAM()->servo_force;
			uint16_t postion = SYSTEM_PARAM()->pos_limit_max;
			if(value == 1)
			{
				cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_OPEN);
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_STOP) /*急停 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING && (value == 0 || value == 1))
		{
			if(value == 1)
			{
				return cmd_servo_stop_init();
			}
		}
	}
	else if(addr == MB_REG_ERR_CLE)/*清除错误 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING && (value == 0 || value == 1))
		{
			if(value == 1)
			{
				fml_motor_reg_wirte(0x0F, 0x01, 3); //清除关节错误
			}
			return RM_OK;
		}
	}
	else if(addr == MB_REG_SET_POS) /*设置夹爪开口度 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			uint16_t speed = SYSTEM_PARAM()->servo_speed;
			uint16_t force = SYSTEM_PARAM()->servo_force;
			uint16_t postion = value;
			
			uint8_t state = E_STATE_PLAN_OPEN;;
			if(postion < SERVO_INFO()->open_range)
			{
				state = E_STATE_PLAN_CLOSE;
				motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
			}
			return cmd_servo_plan_init(speed, force, postion, state);
		}
	}
	else if(addr == MB_REG_SET_SPEED) /*设置夹取速度 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= RM_SPEED_MIN && value <= RM_SPEED_MAX) 
			{
				SYSTEM_PARAM()->servo_speed = value;
				fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SET_SPEED, value); //速度
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_SET_FORCE) /*设置夹取力度 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= RM_FORCE_MIN && value <= RM_FORCE_MAX) 
			{
				SYSTEM_PARAM()->servo_force = value;
				fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SET_FORCE, value); //力度
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_POS_MAX) /*设置最大开口度 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= RM_POS_MIN && value <= RM_POS_MAX && value > SYSTEM_PARAM()->pos_limit_min) 
			{
				SYSTEM_PARAM()->pos_limit_max = value;
				fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_MAX, value); //最大限位
				return RM_OK;
			}
		}
	}
	else if(addr == MB_REG_POS_MIN) /*设置最小开口度 (rw)*/
	{
		value = (buf[4] << 8) | buf[5];
		if(func == E_WRITE_SINGLE_HOLDING)
		{
			if(value >= RM_POS_MIN && value <= RM_POS_MAX && value < SYSTEM_PARAM()->pos_limit_max) 
			{
				SYSTEM_PARAM()->pos_limit_min = value;
				fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_MIN, value);
				return RM_OK;
			}
		}
	}
	return RM_ERROR;
}

/*标准modbusrtu协议处理*/
void apl_init_protocol_handel(void)
{
	DEV_ADDR = SYSTEM_PARAM()->device_id;
	
	SERVO_INFO()->enable = 0;
	SERVO_INFO()->timeout = 0;
	SERVO_INFO()->sys_error = 0;
	SERVO_INFO()->force_targ = 1100;
	SERVO_INFO()->bak_state = E_STATE_STOP_NONE;
	
//	g_dio_state[0].pin_mode = E_DIO_MODE_IN;
//	g_dio_state[1].pin_mode = E_DIO_MODE_IN;
	
	mbCb.wirte_coils_cb = mb_wirte_coils_cb;
	mbCb.wirte_holding_cb = mb_wirte_holding_cb;
	
	fml_modbus_param_init(&mbCb);
}



