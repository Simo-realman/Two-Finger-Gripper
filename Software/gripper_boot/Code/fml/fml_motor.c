#include "fml_motor.h"
#include "cantype.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "fml_modbus.h"
#include "fml_storage.h"

#define DBG_TAG "motor."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

s_motor_state motor_state = {0};

uint32_t g_motor_can_id = 0x07;

extern TRANS_QUEUE qFdcan1Send;
extern TRANS_QUEUE qFdcan1Recv;

s_motor_state *SERVO_INFO(void)
{
	return &motor_state;
}

//0x0* 驱动器状态和参数配置
void _parse_read_reg_sys(uint8_t reg_sub, uint8_t *pData)
{
	if(reg_sub == SYS_ID)//查询关节ID
	{
		//LOG_D("joint id:%d\r\n", );
	}
	else if(reg_sub == SYS_MODEL_TYPE)//查询关节型号
	{
	
	}
	else if(reg_sub == SYS_FW_VERSION)//查询版本号
	{
	
	}
}
//0x1* 当前驱动器电流、速度、位置等控制信息
void _parse_read_reg_cur(uint8_t reg_sub, uint8_t *pData)
{
	if(reg_sub == CUR_CUR_L)
	{
//		motor_state.current = (float)(BUILD_INT32(pData[2], pData[3], pData[4], pData[5]));
//		motor_state.speed = (float)(BUILD_INT32(pData[6], pData[7], pData[8], pData[9]))*100.0;
//		motor_state.postion = (float)(BUILD_INT32(pData[10], pData[11], pData[12], pData[13]))*360/ENCODER_CNT;
		motor_state.current_real = (int)abs((BUILD_INT32(pData[2], pData[3], pData[4], pData[5])));
		motor_state.speed_real = (int)(BUILD_INT32(pData[6], pData[7], pData[8], pData[9]))*100.0;
		motor_state.pos_real = (int)(BUILD_INT32(pData[10], pData[11], pData[12], pData[13]))*360/ENCODER_CNT;
		motor_state.pos_real = (int)((float)motor_state.pos_real/60.0*65);
		fml_motor_limit_check(); //检测阈值状态
	}
	else if(reg_sub == CUR_CAL)//标定状态
	{
	
	}
}
//0x2* 电机模块 及关节全球唯一ID
void _parse_read_reg_mot(uint8_t reg_sub, uint8_t *pData)
{
	
}
//0x3* 关节目标电流、速度、位置等控制信息
void _parse_read_reg_tag(uint8_t reg_sub, uint8_t *pData)
{
	
}

//0x4* 关节速度、加速度、位置等限制信息。
void _parse_read_reg_lit(uint8_t reg_sub, uint8_t *pData)
{
	
}

//0x5* 关节PID配置信息
void _parse_read_reg_sev(uint8_t reg_sub, uint8_t *pData)
{
	
}
//0x6* 关节抱闸信息
void _parse_read_reg_brk(uint8_t reg_sub, uint8_t *pData)
{
	
}
//0x7* 编码器模块相关信息和DRV器件状态信息
void _parse_read_reg_enc(uint8_t reg_sub, uint8_t *pData)
{
	
}
void _parse_write_reg_sys(uint8_t reg_sub, uint8_t *pData)
{

}
//伺服控制指令响应
void _parse_servo_cmd_resp(uint8_t id, uint8_t *pData)
{
	motor_state.error = BUILD_UINT16(pData[14], pData[15]);
	motor_state.en = pData[12];
	
//	motor_state.current = (float)(BUILD_INT32(pData[0], pData[1], pData[2], pData[3]));
//	motor_state.speed = (float)(BUILD_INT32(pData[4], pData[5], pData[6], pData[7]))*100.0;
//	motor_state.postion = (float)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360/ENCODER_CNT;
	motor_state.current_real = (int)abs((BUILD_INT32(pData[0], pData[1], pData[2], pData[3])));
	motor_state.speed_real = (int)(BUILD_INT32(pData[4], pData[5], pData[6], pData[7]))*100.0;
	motor_state.pos_real = (int)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360/ENCODER_CNT;
	motor_state.pos_real = (int)((float)motor_state.pos_real/60.0*65);
	fml_motor_limit_check(); //检测阈值状态
}
//伺服状态响应
void _parse_servo_state_resp(uint8_t id, uint8_t *pData)
{
	motor_state.en = pData[6];
	motor_state.error = BUILD_UINT16(pData[0], pData[1]);
	motor_state.voltage_real = (int)(BUILD_INT16(pData[2], pData[3]))/100.0;
	motor_state.temp_real = (int)(BUILD_INT16(pData[4], pData[5]))/10.0;
	motor_state.pos_real = (int)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360.0/ENCODER_CNT;
	motor_state.pos_real = (int)((float)motor_state.pos_real/60.0*65); //角度转位置
	motor_state.current_real = (int)abs((BUILD_INT32(pData[12], pData[13], pData[14], pData[15]))); //mA
	LOG_D("V:%d P:%d C:%d T:%d e:%04x en:%d\r\n", motor_state.voltage_real, motor_state.pos_real, motor_state.current_real, motor_state.temp_real, motor_state.error, motor_state.en);
	fml_motor_limit_check(); //检测阈值状态
//	motor_state.voltage = (float)(BUILD_INT16(pData[2], pData[3]))/100.0;
//	motor_state.temp = (float)(BUILD_INT16(pData[4], pData[5]))/10.0;
//	motor_state.postion = (float)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360.0/ENCODER_CNT;
//	motor_state.current = (float)(BUILD_INT32(pData[12], pData[13], pData[14], pData[15])); //mA
//	LOG_D("V:%.1f P:%.1f C:%.1f T:%.1f e:%04x en:%d\r\n", motor_state.voltage, motor_state.postion, motor_state.current, motor_state.temp, motor_state.error, motor_state.en);
}

void fml_fdcan_data_parse(s_queue_can_data *pData)
{
	uint8_t id, cmd, reg_pre, reg_sub;
	uint32_t can_id = pData->id&0x0FFF; //canID
	uint8_t frame_type = (can_id>>8)&0xFF; //寄存器类型
	
	if(frame_type == 1) //寄存器读写响应
	{
		id = can_id - 0x100;
		cmd = pData->data[0];
		reg_pre = (pData->data[1]&0xF0)>>4; //寄存器主
		reg_sub = pData->data[1]&0x0F; //寄存器
		
		if(cmd == E_CMD_RD) //读寄存器
		{
			if(reg_pre == E_REG_MAIN_SYS)
			{
				_parse_read_reg_sys(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_CUR)
			{
				_parse_read_reg_cur(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_MOT)
			{
				_parse_read_reg_mot(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_TAG)
			{
				_parse_read_reg_tag(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_LIT)
			{
				_parse_read_reg_lit(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_SEV)
			{
				_parse_read_reg_sev(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_BRK)
			{
				_parse_read_reg_brk(reg_sub, pData->data);
			}
			else if(reg_pre == E_REG_MAIN_ENC)
			{
				_parse_read_reg_enc(reg_sub, pData->data);
			}
		}
		else if(cmd == E_CMD_WR) //写寄存器
		{
			_parse_write_reg_sys(reg_sub, pData->data);
		}
	}
	else if(frame_type == 5) //伺服控制响应
	{
		id = can_id - 0x500;
		_parse_servo_cmd_resp(id, pData->data);
	}
	else if(frame_type == 7) //状态响应
	{
		id = can_id - 0x700;
		_parse_servo_state_resp(id, pData->data);
	}
}

//数据发送
void fml_motor_send(s_queue_can_data *pData)
{
	if(!FullQueue(&qFdcan1Send))
	{
		Enqueue(&qFdcan1Send, pData);
	}
}

//写寄存器
void fml_motor_reg_wirte(uint8_t index, uint16_t data, uint16_t len)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id;
	send_data.len = len;
	send_data.data[0] = E_CMD_WR;
	send_data.data[1] = index;
	send_data.data[2] = LO_UINT16(data);
	send_data.data[3] = HI_UINT16(data);
	
	fml_motor_send(&send_data);
}

//读寄存器
void fml_motor_reg_read(uint8_t index, uint16_t num)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id;
	send_data.len = 3;
	send_data.data[0] = E_CMD_RD;
	send_data.data[1] = index;
	send_data.data[2] = num;
	
	fml_motor_send(&send_data);
}

//给关节发送数据
void fml_motor_frame_send(uint8_t *data, uint16_t len)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id;
	send_data.len = len;
	if(len > sizeof(send_data.data))
		return;
	memcpy(send_data.data, data, len);
	
	fml_motor_send(&send_data);
}

//关节限位配置
void fml_motor_limit_cmd(uint8_t index, uint16_t data)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id;
	send_data.data[0] = E_CMD_WR;
	send_data.data[1] = index;
	uint8 sub_index = LO_UINT8(index);
	if(sub_index == LIT_MAX_CURRENT) 
	{
		send_data.len = 4;
		send_data.data[2] = LO_UINT16(data);
		send_data.data[3] = HI_UINT16(data);
	}
	else if(sub_index == LIT_MAX_SPEED) 
	{
		
	}
	else if(sub_index == LIT_MAX_CURRENT) 
	{
		
	}
	else if(sub_index == LIT_MAX_ACC) 
	{
		
	}
	else if(sub_index == LIT_MAX_DEC) 
	{
		
	}
	else if(sub_index == LIT_MIN_POS_L) 
	{
		
	}
	else if(sub_index == LIT_MAX_POS_L) 
	{
		
	}
	
	fml_motor_send(&send_data);
}

//关节位置伺服控制
void fml_motor_servo_pos_cmd(float pos)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id | SERVER_POS_CTRL_ID;
	send_data.len = 4;	
	send_data.data[0] = (uint8_t)((int)(pos*ENCODER_CNT/360));
	send_data.data[1] = (uint8_t)((int)(pos*ENCODER_CNT/360) >> 8);
	send_data.data[2] = (uint8_t)((int)(pos*ENCODER_CNT/360) >> 16);
	send_data.data[3] = (uint8_t)((int)(pos*ENCODER_CNT/360) >> 24);
	
	fml_motor_send(&send_data);
}

//关节速度伺服控制
void fml_motor_servo_speed_cmd(float speed)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id | SERVER_SPD_CTRL_ID;
	send_data.len = 4;	
	send_data.data[0] = (uint8_t)((int)(speed*100));
	send_data.data[1] = (uint8_t)((int)(speed*100) >> 8);
	send_data.data[2] = (uint8_t)((int)(speed*100) >> 16);
	send_data.data[3] = (uint8_t)((int)(speed*100) >> 24);
	
	fml_motor_send(&send_data);
}

//关节电流伺服控制
void fml_motor_servo_current_cmd(float current)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id | SERVER_CUR_CTRL_ID;
	send_data.len = 4;	
	send_data.data[0] = (uint8_t)((int)(current));
	send_data.data[1] = (uint8_t)((int)(current) >> 8);
	send_data.data[2] = (uint8_t)((int)(current) >> 16);
	send_data.data[3] = (uint8_t)((int)(current) >> 24);
	
	fml_motor_send(&send_data);
}

//关节伺服状态查询
void fml_motor_servo_state_cmd(void)
{
	s_queue_can_data send_data = {0};
	
	send_data.id = g_motor_can_id | SERVER_STATE_ASK_ID;
	send_data.len = 0;		
	fml_motor_send(&send_data);
}

//阈值检测
void fml_motor_limit_check(void)
{
	//寄存器初始化
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_ERROR_CODE, motor_state.error);
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SERVO_STATE, motor_state.run_state);
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_TEMP_REAL, motor_state.temp_real);
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_CURRENT_REAL, motor_state.current_real);
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_REAL, abs(motor_state.pos_real));
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_FORCE_REAL, motor_state.force_real);
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_VOLTAGE_REAL, abs(motor_state.voltage_real));
	
	if(motor_state.current_real >= RM_CURRENT_MAX || motor_state.pos_real > RM_POS_MAX  || motor_state.pos_real < RM_POS_MIN || motor_state.error != 0)
	{
		LOG_D("limit stop %d %d %d\r\n",motor_state.current_real, motor_state.pos_real, motor_state.error);
		motor_state.run_state = E_STATE_STOP; //停止运动
	}
}


