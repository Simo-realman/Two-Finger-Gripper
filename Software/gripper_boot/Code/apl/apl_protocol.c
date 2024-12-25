#include "apl_protocol.h"

#include "fml_motor.h"
#include "fml_modbus.h"
#include "fml_storage.h"
#include "fml_queue.h"
#include "fml_invoke_algo.h"
#include "common.h"

#define DBG_TAG "pro."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

s_mb_rtu_cb mbCb;
static uint8_t DEV_ADDR = 0x01;

extern TRANS_QUEUE qUart1Recv;
extern TRANS_QUEUE qUart1Send;

void cmd_para_save_handle(uint8_t *buf, uint16_t len);
void cmd_para_default_handle(uint8_t *buf, uint16_t len);
void cmd_dev_id_set_handle(uint8_t *buf, uint16_t len);
void cmd_dev_id_get_handle(uint8_t *buf, uint16_t len);
void cmd_dev_baud_set_handle(uint8_t *buf, uint16_t len);
void cmd_move_catch_xg_handle(uint8_t *buf, uint16_t len);
void cmd_move_release_handle(uint8_t *buf, uint16_t len);
void cmd_set_eg_para_handle(uint8_t *buf, uint16_t len);
void cmd_read_eg_para_handle(uint8_t *buf, uint16_t len);
void cmd_read_eg_state_handle(uint8_t *buf, uint16_t len);
void cmd_move_stop_handle(uint8_t *buf, uint16_t len);
void cmd_error_clear_handle(uint8_t *buf, uint16_t len);
void cmd_move_catch2_xg_handle(uint8_t *buf, uint16_t len);
void cmd_read_eg_runstate_handle(uint8_t *buf, uint16_t len);
void cmd_set_seekpos_handle(uint8_t *buf, uint16_t len);
void cmd_read_actpos_handle(uint8_t *buf, uint16_t len);
void cmd_seekpos_speed_force_handle(uint8_t *buf, uint16_t len);

#define MAX_485_CMD_NUM 17
s_485_protol_handle protol_485_handle[MAX_485_CMD_NUM] = {
	{CMD_MC_PARA_SAVE, cmd_para_save_handle},
	{CMD_MC_PARA_DEFAULT, cmd_para_default_handle},
	{CMD_MC_PARA_ID_SET, cmd_dev_id_set_handle},
	{CMD_MC_PARA_ID_GET, cmd_dev_id_get_handle},
	{CMD_MC_PARA_BAUD_SET, cmd_dev_baud_set_handle},
	{CMD_MC_MOVE_CATCH_XG, cmd_move_catch_xg_handle},
	{CMD_MC_MOVE_RELEASE, cmd_move_release_handle},
	{CMD_MC_SET_EG_PARA, cmd_set_eg_para_handle},
	{CMD_MC_READ_EG_PARA, cmd_read_eg_para_handle},
	{CMD_MC_READ_EG_STATE, cmd_read_eg_state_handle},
	{CMD_MC_MOVE_STOPHERE, cmd_move_stop_handle},
	{CMD_MC_ERROR_CLR, cmd_error_clear_handle},
	{CMD_MC_MOVE_CATCH2_XG, cmd_move_catch2_xg_handle},
	{CMD_MC_READ_EG_RUNSTATE, cmd_read_eg_runstate_handle},
	{CMD_MC_SEEKPOS, cmd_set_seekpos_handle},
	{CMD_MC_READ_ACTPOS, cmd_read_actpos_handle},
	{CMD_MC_SEEKPOS_SPEED_FORCE, cmd_seekpos_speed_force_handle}
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
	
	if(buf[2] != DEV_ADDR)
	{
		return;
	} 
	else if(buf[2] == 0xFF)
	{
		for(i = 0; i < MAX_485_CMD_NUM; i++)
		{
			if(protol_485_handle[i].cmd == buf[4])
			{
				protol_485_handle[i].func(buf, len);
			}
		}
	}
	else
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

//响应
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
	
	if(!FullQueue(&qUart1Send))
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
	if(!FullQueue(&qUart1Send))
	{
		Enqueue(&qUart1Send, &send_data);
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
		cmd_state_resp(CMD_MC_PARA_ID_SET, CMD_MC_OK);
	}
	else
	{
		cmd_state_resp(CMD_MC_PARA_ID_SET, CMD_MC_ERR);
	}
	LOG_D("set id:%d\r\n", SYSTEM_PARAM()->device_id);
}

//获取设备ID
void cmd_dev_id_get_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[1] = {0};
	
	data[0] = DEV_ADDR;
	cmd_data_resp(CMD_MC_PARA_ID_GET, data, 1);
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
	int speed = BUILD_UINT16(buf[5], buf[6]);
	int force = BUILD_UINT16(buf[7], buf[8]);
	
	LOG_D("speed:%d force:%d\r\n", speed, force);
	
	if(speed > RM_SPEED_MAX || speed < RM_SPEED_MIN || force > RM_FORCE_MAX || force < RM_FORCE_MIN)
	{
		cmd_state_resp(CMD_MC_MOVE_CATCH_XG, CMD_MC_ERR);
		return;
	}
	fml_my_algo_init(SERVO_INFO()->pos_real, 0, speed);
	SERVO_INFO()->run_state = E_STATE_RUNING;
	/*
	*
	*/
	cmd_state_resp(CMD_MC_MOVE_CATCH_XG, CMD_MC_OK);
}

//以设置的速度和力控阈值持续夹取
void cmd_move_catch2_xg_handle(uint8_t *buf, uint16_t len)
{
	int speed = BUILD_UINT16(buf[5], buf[6]);
	int force = BUILD_UINT16(buf[7], buf[8]);
	LOG_D("speed:%d force:%d\r\n", speed, force);
	if(speed > RM_SPEED_MAX || speed < RM_SPEED_MIN || force > RM_FORCE_MAX || force < RM_FORCE_MIN)
	{
		cmd_state_resp(CMD_MC_MOVE_CATCH2_XG, CMD_MC_ERR);
		return;
	}
	fml_my_algo_init(SERVO_INFO()->pos_real, 0, speed);
	SERVO_INFO()->run_state = E_STATE_RUNING;
	/*
	*
	*/
	cmd_state_resp(CMD_MC_MOVE_CATCH2_XG, CMD_MC_OK);
}

//以设置的速度松开
void cmd_move_release_handle(uint8_t *buf, uint16_t len)
{
	int speed = BUILD_UINT16(buf[5], buf[6]);
	LOG_D("speed:%d\r\n", speed);
	if(speed > RM_SPEED_MAX || speed < RM_SPEED_MIN)
	{
		cmd_state_resp(CMD_MC_MOVE_RELEASE, CMD_MC_ERR);
		return;
	}
	/*
	*
	*/
	fml_my_algo_init(SERVO_INFO()->pos_real, 60, speed);
	SERVO_INFO()->run_state = E_STATE_RUNING;
	cmd_state_resp(CMD_MC_MOVE_RELEASE, CMD_MC_OK);
}

//急停
void cmd_move_stop_handle(uint8_t *buf, uint16_t len)
{
	SERVO_INFO()->run_state = E_STATE_STOP;
	/*
	*
	*/
	cmd_state_resp(CMD_MC_MOVE_STOPHERE, CMD_MC_OK);
}

//清除错误
void cmd_error_clear_handle(uint8_t *buf, uint16_t len)
{
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

//指定夹爪开口度
void cmd_set_seekpos_handle(uint8_t *buf, uint16_t len)
{
	uint16_t pos = BUILD_UINT16(buf[5], buf[6]);
	uint16_t angle_tag = (uint16_t)((float)(pos)/65.0*60); //位置转角度
	uint16_t angle_now = (uint16_t)((float)SERVO_INFO()->pos_real/65.0*60); //位置转角度
	
	if(pos > RM_POS_MAX || pos < RM_POS_MIN)
	{
		cmd_state_resp(CMD_MC_SEEKPOS, CMD_MC_ERR);
		return;
	}
	
	fml_my_algo_init(angle_now, angle_tag, SYSTEM_PARAM()->servo_speed);
	SERVO_INFO()->run_state = E_STATE_RUNING;
	/*
	*
	*/
	cmd_state_resp(CMD_MC_SEEKPOS, CMD_MC_OK);
}

//读取夹爪开口度
void cmd_read_actpos_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[2] = {0};
	
	data[0] = LO_UINT16(SERVO_INFO()->pos_real);
	data[1] = HI_UINT16(SERVO_INFO()->pos_real);
	
	cmd_data_resp(CMD_MC_READ_ACTPOS, data, 2);
}

//读取夹爪状态
void cmd_read_eg_state_handle(uint8_t *buf, uint16_t len)
{
	//暂不支持
}

//读取夹爪运行状态
void cmd_read_eg_runstate_handle(uint8_t *buf, uint16_t len)
{
	uint8_t data[13] = {0};
	
	data[0] = SERVO_INFO()->run_state;
	data[1] = SERVO_INFO()->error;
	data[2] = SERVO_INFO()->temp_real;
	data[3] = LO_UINT16(SERVO_INFO()->pos_real);
	data[4] = HI_UINT16(SERVO_INFO()->pos_real);
	data[5] = LO_UINT16(SERVO_INFO()->force_real);
	data[6] = HI_UINT16(SERVO_INFO()->force_real);
	data[7] = LO_UINT16(SERVO_INFO()->speed_real);
	data[8] = HI_UINT16(SERVO_INFO()->speed_real);
	data[9] = LO_UINT16(SERVO_INFO()->voltage_real);
	data[10] = HI_UINT16(SERVO_INFO()->voltage_real);
	data[11] = LO_UINT16(SERVO_INFO()->current_real);
	data[12] = HI_UINT16(SERVO_INFO()->current_real);
	cmd_data_resp(CMD_MC_READ_EG_RUNSTATE, data, 13);
}

//设置夹爪开口度、速度、力控阈值
void cmd_seekpos_speed_force_handle(uint8_t *buf, uint16_t len)
{
	int pos = BUILD_UINT16(buf[5], buf[6]);
	int speed = BUILD_UINT16(buf[7], buf[8]);
	int force = BUILD_UINT16(buf[9], buf[10]);
	LOG_D("servo:%d %d %d\r\n", pos, speed, force);
	if(speed > RM_SPEED_MAX || speed < RM_SPEED_MIN || force > RM_FORCE_MAX || force < RM_FORCE_MIN || pos > RM_POS_MAX || pos < RM_POS_MIN)
	{
		cmd_state_resp(CMD_MC_SEEKPOS_SPEED_FORCE, CMD_MC_ERR);
		return;
	}
	fml_my_algo_init(SERVO_INFO()->pos_real, pos, speed);
	SERVO_INFO()->run_state = E_STATE_RUNING;
	/*
	*
	*/
	
	cmd_state_resp(CMD_MC_SEEKPOS_SPEED_FORCE, CMD_MC_OK);
}



//写线圈处理
uint8_t mb_wirte_coils_cb(uint8_t *buf, uint16_t len)
{
	
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
			if(value == 0)
			{
				/*
					一次夹取
				*/
				return RM_OK;
			}
			else if(value == 1)
			{
				/*
					持续夹取
				*/
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
				/*
					急停
				*/
			}
			return RM_OK;
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
			if(value >= RM_POS_MIN && value <= RM_POS_MAX) 
			{
				/*
					立即控制
				*/
				return RM_OK;
			}
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
				fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_MIN, value); //最小限位
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
	
	mbCb.wirte_coils_cb = mb_wirte_coils_cb;
	mbCb.wirte_holding_cb = mb_wirte_holding_cb;
	
	fml_modbus_param_init(&mbCb);
}



