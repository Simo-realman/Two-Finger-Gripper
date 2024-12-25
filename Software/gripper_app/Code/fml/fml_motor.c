#include "fml_motor.h"
#include "cantype.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "fml_modbus.h"
#include "fml_storage.h"
#include "fml_invoke_algo.h"
#include "gripper_algo.h"

#define DBG_TAG "motor."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

s_motor_state motor_state = {0};
uint32_t g_motor_can_id = 0x01;

/****广播帧****/
#define RESP_BACK_FLAG   0x07 //值返回关节id=1的数据
#define RESP_ID_RW   		 (0xF0|(g_motor_can_id+1))
#define RESP_ID_SERVO    (0x80|(g_motor_can_id+1))

extern filter_st curr_filter; //滤波
extern TRANS_QUEUE qFdcan1Send;
extern TRANS_QUEUE qFdcan1Recv;

s_motor_state *SERVO_INFO(void)
{
	return &motor_state;
}

//modbus寄存器更新
void fml_motor_modbus_update(void)
{
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_REAL, (uint16_t)motor_state.open_range); //开口度
#ifdef RM_HIGHT_FORCE_ENABLE	
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_FORCE_REAL, (uint16_t)(motor_state.force/2.0)); //力
#else 
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_FORCE_REAL, (uint16_t)motor_state.force); //力
#endif	
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_ERROR_CODE, (uint16_t)motor_state.error); //错误码
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SERVO_STATE, motor_state.bak_state); //运行状态
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_TEMP_REAL, (uint16_t)motor_state.temp); //温度
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_CURRENT_REAL, (uint16_t)motor_state.current); //电流
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_VOLTAGE_REAL, (uint16_t)motor_state.voltage); //电压
}

//阈值检测
uint8_t fml_motor_limit_check(void)
{
	if((motor_state.force > motor_state.force_targ) && (motor_state.current < 0)) //力超过限位
	{
		LOG_E("force over !!! %.1f\r\n", motor_state.force);
		if(motor_state.run_state == E_STATE_RUNING) //夹持到物体
		{
			fml_radio_servo_pos_none_cmd(motor_state.postion + 2, 0, 0); //主动张开快速释放力
			
#ifdef RM_HIGHT_FORCE_ENABLE			
			motor_state.force_range = (motor_state.force_targ - RM_CURRENT_OFFSET)*2; //设置力控阈值0-1000 对应0-2000
#else
			motor_state.force_range = motor_state.force_targ - RM_CURRENT_OFFSET; //设置力控阈值,防止发热0-1000
#endif
			if(motor_state.force_range < RM_FORCE_CTRL_MIN) //最小力控值
			{
				motor_state.force_range = RM_FORCE_CTRL_MIN;
			}
			motor_state.force_enable = RM_TRUE; //开启力控
		}
		return 0;
	}
	
	if(motor_state.error && (motor_state.error != E_ERR_CMD_POS)) //错误
	{
		LOG_E("error !!! %04x\r\n", motor_state.error);
		return 0;
	}
	if(motor_state.postion > RM_ANGLE_MAX + 1 || motor_state.postion < RM_ANGLE_MIN - 1) //角度限位
	{
		LOG_E("postion over !!! %.1f\r\n", motor_state.postion);
		return 0;
	}
	
	return 1;
}

//广播帧解析
void fml_radio_data_parse(s_queue_can_data *pData)
{
	uint8_t len = 0;
	uint32_t can_id = pData->id&0x0FFF; //canID
	
	if(can_id == RESP_ID_RW) //读写
	{
		if(pData->data[0] == E_CMD_WR) //写数据响应
		{
			//LOG_D("write:%02x state:%d\r\n", pData->data[1], pData->data[2]);
		}
		else //读数据响应
		{
			if(pData->data[1] == E_CMD_RD)
			{
				len = pData->data[0];
				//LOG_D("read:%02x len:%d value:%d\r\n", pData->data[2], len, pData->data[3]);
			}
		}
	}
	else if(can_id == RESP_ID_SERVO) //伺服指令和状态读取
	{
		motor_state.enable = 1; //数据有效
		motor_state.timeout = 0;
		motor_state.current = (float)(BUILD_INT32(pData->data[0], pData->data[1], pData->data[2], pData->data[3])); //当前电流mA
		motor_state.speed = (float)(BUILD_INT32(pData->data[4], pData->data[5], pData->data[6], pData->data[7])); // 当前速度0.02RPM 
		motor_state.speed = (motor_state.speed/10.0)/SPEED_CMD_UNIT; //当前速度°/S
		motor_state.postion = (float)(BUILD_INT32(pData->data[8], pData->data[9], pData->data[10], pData->data[11]))*360.0/ENCODER_CNT; //当前角度
		motor_state.error = BUILD_UINT16(pData->data[12], pData->data[13]); //错误码
		motor_state.voltage = (float)(BUILD_INT16(pData->data[14], pData->data[15]))/100.0; //当前电压
		motor_state.temp = (float)(BUILD_INT16(pData->data[16], pData->data[17]))/10.0; //当前温度
		motor_state.en = pData->data[18]; //使能状态
		motor_state.range_mm = gripper_kine(HOME_THETA, motor_state.postion); //开口度0-65mm
		motor_state.open_range = (uint16_t)(RM_POS_MAX*(motor_state.range_mm/RM_RANGE_MAX)); //开口度0-1000mm
		
		fml_motor_modbus_update(); //更新modbus数据
		
		//update_motor_real_position(motor_state.postion); //更新当前角度
		
		//LOG_D("f:%.1f pos:%.d dg:%.1f cur:%.1f spd:%.1f vol:%.1f tmp:%.1f err:%04x en:%d\r\n", \
		motor_state.force,\
		motor_state.open_range,\
		motor_state.postion,\
		motor_state.current,\
		motor_state.speed,\
		motor_state.voltage,\
		motor_state.temp,\
		motor_state.error,\
		motor_state.en);
	}
}

//广播帧写寄存器
void fml_radio_servo_reg_write(uint8_t index, uint16_t data, uint16_t len)
{
	s_queue_can_data send_data = {0};
	send_data.id = SERVER_WR_CTRL_RADIO_ID;
	send_data.len = 64;
	send_data.data[0 + 7 * (g_motor_can_id - 1)] = len;
	send_data.data[1 + 7 * (g_motor_can_id - 1)] = E_CMD_WR;
	send_data.data[2 + 7 * (g_motor_can_id - 1)] = index;
	send_data.data[3 + 7 * (g_motor_can_id - 1)] = LO_UINT16(data);
	send_data.data[4 + 7 * (g_motor_can_id - 1)] = HI_UINT16(data);
	send_data.data[5 + 7 * (g_motor_can_id - 1)] = 0;
	send_data.data[6 + 7 * (g_motor_can_id - 1)] = 0;
	send_data.data[63] = RESP_BACK_FLAG;
	fml_motor_send(&send_data);
}

//广播帧写寄存器-用于设置关节ID
void fml_radio_servo_reg_write_any(uint8_t id, uint8_t index, uint16_t data, uint16_t len)
{
	s_queue_can_data send_data = {0};
	send_data.id = SERVER_WR_CTRL_RADIO_ID;
	send_data.len = 64;
	send_data.data[0 + 7 * (id - 1)] = len;
	send_data.data[1 + 7 * (id - 1)] = E_CMD_WR;
	send_data.data[2 + 7 * (id - 1)] = index;
	send_data.data[3 + 7 * (id - 1)] = LO_UINT16(data);
	send_data.data[4 + 7 * (id - 1)] = HI_UINT16(data);
	send_data.data[5 + 7 * (id - 1)] = 0;
	send_data.data[6 + 7 * (id - 1)] = 0;
	send_data.data[63] = RESP_BACK_FLAG;
	fml_motor_send(&send_data);
}

//广播帧读寄存器
void fml_radio_servo_reg_read(uint8_t index, uint16_t num)
{
	s_queue_can_data send_data = {0};
	send_data.id = SERVER_WR_CTRL_RADIO_ID;
	send_data.len = 64;
	send_data.data[0 + 7 * (g_motor_can_id - 1)] = 4;
	send_data.data[1 + 7 * (g_motor_can_id - 1)] = E_CMD_RD;
	send_data.data[2 + 7 * (g_motor_can_id - 1)] = index;
	send_data.data[3 + 7 * (g_motor_can_id - 1)] = num;
	send_data.data[4 + 7 * (g_motor_can_id - 1)] = 0xFF;
	send_data.data[5 + 7 * (g_motor_can_id - 1)] = 0xFF;
	send_data.data[6 + 7 * (g_motor_can_id - 1)] = 0xFF;
	send_data.data[63] = RESP_BACK_FLAG;
	fml_motor_send(&send_data);
}

//广播关节状态查询
void fml_radio_servo_state_cmd(void)
{
	s_queue_can_data send_data = {0};
	send_data.id = SERVER_STATE_RADIO_ID;
	send_data.len = 24;	
	send_data.data[23] = RESP_BACK_FLAG;
	fml_motor_send(&send_data);
}


//广播关节位置伺服控制
void fml_radio_servo_pos_cmd(float pos, float speed, uint8_t speed_enable)
{
	if(fml_motor_limit_check() == 0)
	{
		motor_state.run_state = E_STATE_NONE;
		motor_state.bak_state = E_STATE_STOP_NONE;
		return;
	}
	
	uint8_t feedback_flag = RESP_BACK_FLAG, flag = 0;
	s_queue_can_data send_data = {0};

	int postion = (int)(pos*ENCODER_CNT_L);
	int spd = (int)(speed*SPEED_CMD_UNIT);
	
	send_data.id = SERVER_POSE_CTRL_RADIO_ID;
	send_data.len = 64;	
	send_data.data[0 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion);
	send_data.data[1 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 8);
	send_data.data[2 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 16);
	send_data.data[3 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 24); //位置
	send_data.data[4 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd);
	send_data.data[5 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd >> 8); //速度前馈
	send_data.data[6 + 8 * (g_motor_can_id - 1)] = 0;
	send_data.data[7 + 8 * (g_motor_can_id - 1)] = 0; //电流前馈
	
	//打包与回复使能
	feedback_flag = feedback_flag & 0x0f;
	flag = flag | feedback_flag;
	flag = flag | speed_enable << 6; //速度前馈使能
	flag = flag | 0x00 << 4; //电流前馈使能
	send_data.data[63] = flag;

	fml_motor_send(&send_data);
}

//广播关节位置伺服控制-位置限制用于力位控制
void fml_radio_servo_pos_force_cmd(float pos, float speed, uint8_t speed_enable)
{	
	if(motor_state.postion > RM_ANGLE_MAX || motor_state.postion < RM_ANGLE_MIN) //角度限位
	{
		motor_state.force_enable = RM_FALSE; //关闭力控
		return;
	}
	uint8_t feedback_flag = RESP_BACK_FLAG, flag = 0;
	s_queue_can_data send_data = {0};

	int postion = (int)(pos*ENCODER_CNT_L);
	int spd = (int)(speed*SPEED_CMD_UNIT);
	
	send_data.id = SERVER_POSE_CTRL_RADIO_ID;
	send_data.len = 64;	
	send_data.data[0 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion);
	send_data.data[1 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 8);
	send_data.data[2 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 16);
	send_data.data[3 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 24); //位置
	send_data.data[4 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd);
	send_data.data[5 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd >> 8); //速度前馈
	send_data.data[6 + 8 * (g_motor_can_id - 1)] = 0;
	send_data.data[7 + 8 * (g_motor_can_id - 1)] = 0; //电流前馈
	
	//打包与回复使能
	feedback_flag = feedback_flag & 0x0f;
	flag = flag | feedback_flag;
	flag = flag | speed_enable << 6; //速度前馈使能
	flag = flag | 0x00 << 4; //电流前馈使能
	send_data.data[63] = flag;

	fml_motor_send(&send_data);
}

//广播关节位置伺服控制-不做任何限制
void fml_radio_servo_pos_none_cmd(float pos, float speed, uint8_t speed_enable)
{	
	uint8_t feedback_flag = RESP_BACK_FLAG, flag = 0;
	s_queue_can_data send_data = {0};

	int postion = (int)(pos*ENCODER_CNT_L);
	int spd = (int)(speed*SPEED_CMD_UNIT);
	
	send_data.id = SERVER_POSE_CTRL_RADIO_ID;
	send_data.len = 64;	
	send_data.data[0 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion);
	send_data.data[1 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 8);
	send_data.data[2 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 16);
	send_data.data[3 + 8 * (g_motor_can_id - 1)] = (uint8_t)(postion >> 24); //位置
	send_data.data[4 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd);
	send_data.data[5 + 8 * (g_motor_can_id - 1)] = (uint8_t)(spd >> 8); //速度前馈
	send_data.data[6 + 8 * (g_motor_can_id - 1)] = 0;
	send_data.data[7 + 8 * (g_motor_can_id - 1)] = 0; //电流前馈
	
	//打包与回复使能
	feedback_flag = feedback_flag & 0x0f;
	flag = flag | feedback_flag;
	flag = flag | speed_enable << 6; //速度前馈使能
	flag = flag | 0x00 << 4; //电流前馈使能
	send_data.data[63] = flag;

	fml_motor_send(&send_data);
}


//电流信号滤波
void fml_current_filter(void)
{
	motor_state.current =  utils_second_order_filter(motor_state.current, &curr_filter); //电流滤波
	motor_state.force = abs(motor_state.current); //当前加持力
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
		motor_state.current = (float)(BUILD_INT32(pData[2], pData[3], pData[4], pData[5]));
		motor_state.speed = (float)(BUILD_INT32(pData[6], pData[7], pData[8], pData[9]))*100.0;
		motor_state.postion = (float)(BUILD_INT32(pData[10], pData[11], pData[12], pData[13]))*360/ENCODER_CNT;
		
		fml_motor_limit_check(); //检测阈值状态
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
	motor_state.current = (float)(BUILD_INT32(pData[0], pData[1], pData[2], pData[3]));
	motor_state.speed = (float)(BUILD_INT32(pData[4], pData[5], pData[6], pData[7]))*100.0;
	motor_state.postion = (float)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360/ENCODER_CNT;
	fml_motor_limit_check(); //检测阈值状态
}
//伺服状态响应
void _parse_servo_state_resp(uint8_t id, uint8_t *pData)
{
	motor_state.en = pData[6];
	motor_state.error = BUILD_UINT16(pData[0], pData[1]);
	motor_state.voltage = (float)(BUILD_INT16(pData[2], pData[3]))/100.0;
	motor_state.temp = (float)(BUILD_INT16(pData[4], pData[5]))/10.0;
	motor_state.postion = (float)(BUILD_INT32(pData[8], pData[9], pData[10], pData[11]))*360.0/ENCODER_CNT;
	motor_state.current = (float)(BUILD_INT32(pData[12], pData[13], pData[14], pData[15])); //mA
	
	LOG_D("pos:%.1f cur:%.1f tp:%.1f e:%04x en:%d\r\n", motor_state.postion, motor_state.current, motor_state.temp, motor_state.error, motor_state.en);
	fml_motor_limit_check(); //检测阈值状态
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
	send_data.data[0] = (uint8_t)((int)(pos*ENCODER_CNT_L));
	send_data.data[1] = (uint8_t)((int)(pos*ENCODER_CNT_L) >> 8);
	send_data.data[2] = (uint8_t)((int)(pos*ENCODER_CNT_L) >> 16);
	send_data.data[3] = (uint8_t)((int)(pos*ENCODER_CNT_L) >> 24);
	//LOG_D("%.1f\r\n", pos);
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


