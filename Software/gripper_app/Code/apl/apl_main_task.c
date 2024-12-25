#include "apl_main_task.h"
#include "apl_protocol.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_dma.h"
#include "hal_usart.h"
#include "hal_tim.h"
#include "hal_flash.h"
#include "hal_iwdg.h"
#include "hal_fdcan.h"
#include "fml_storage.h"
#include "fml_queue.h"
#include "fml_motor.h"
#include "fml_modbus.h"
#include "fml_invoke_algo.h"
#include "gripper_algo.h"
#include "cantype.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>
#define DBG_TAG "m.t"
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

#define Q_USART_SEND_SIZE	10
#define Q_USART_RECV_SIZE	10
TRANS_QUEUE qUart1Send = {0};
TRANS_QUEUE qUart1Recv = {0};
#define Q_CAN_SEND_SIZE	10
#define Q_CAN_RECV_SIZE	10
TRANS_QUEUE qFdcan1Send = {0};
TRANS_QUEUE qFdcan1Recv = {0};

float plan_pos_f = 0; //规划位置
float plan_speed_f = 0; //速度前馈
uint8_t g_dio_plan_state = 0; //IO控制开合标志位
uint16_t g_app_version = 105; //版本信息

extern s_io_state g_dio_state[2]; //DIO状态
extern Trajectory g_traj_palan; //算法轨迹规划
extern s_motor_state motor_state; //电机状态
extern system_parameters_t spt; //系统参数
extern uint8_t g_task_cycle_one; //任务标志位
extern uint8_t g_task_cycle_two; //任务标志位
extern uint8_t g_upgrade_enable; //升级超时标志位
extern uint8_t algo_init_flag; //算法初始化标志位
extern PIDController pid; //PID算法

void fdcan_send_task_entry(void);
void fdcan_recv_task_entry(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pData);
void uart_recv_task_entry(uint8_t *pData, uint16_t len);
void dio_plan_sub_task(void);
void fml_dio_init(void);

void queue_create()
{
	CreateQueue(&qUart1Send,Q_USART_SEND_SIZE,sizeof(s_queue_uart_data));
	CreateQueue(&qUart1Recv,Q_USART_RECV_SIZE,sizeof(s_queue_uart_data));
	CreateQueue(&qFdcan1Send,Q_CAN_SEND_SIZE,sizeof(s_queue_can_data));
	CreateQueue(&qFdcan1Recv,Q_CAN_SEND_SIZE,sizeof(s_queue_can_data));
}

void peripheral_init(void)
{
	queue_create(); //队列初始化
	fml_system_param_init(); //参数初始化
	apl_init_protocol_handel(); //协议接口初始化
	fml_filter_init(); //滤波器初始化
	//init_pid(&pid, KP, KI, KD, LIMIT);
	
	__hal_gpio_init();
	__hal_dma_init();
	__hal_fdcan1_init(fdcan_recv_task_entry);
	__hal_uart1_init((e_baud_type)SYSTEM_PARAM()->uart1_baud, uart_recv_task_entry);
	__hal_uart3_init();
	__hal_timer2_init(); //200us
	
	fml_dio_init();
	HAL_Delay(4000); //等待关节启动
	__hal_iwdg_init();
	fml_set_rgb(E_RGB_GREEN);
	LOG_D("ID:%d BAUD:%d POS:%d-%d VN:%d\r\n", SYSTEM_PARAM()->device_id,SYSTEM_PARAM()->uart1_baud,SYSTEM_PARAM()->pos_limit_min,SYSTEM_PARAM()->pos_limit_max,g_app_version);
}

//can数据发送任务
void fdcan_send_task_entry(void)
{
	if(g_task_cycle_one) //200us 发送can数据
	{
		g_task_cycle_one = 0;
		s_queue_can_data send_data = {0};
		if(!EmptyQueue(&qFdcan1Send))
		{
			Dequeue(&qFdcan1Send, &send_data);
			hal_fdcan1_send(send_data.id ,send_data.data, send_data.len);
		}
	}
	if(g_task_cycle_two) //2ms规划周期透传算法
	{
		g_task_cycle_two = 0;
		if(motor_state.run_state == E_STATE_RUNING) //轨迹规划
		{
			if(otg_update(motor_state.postion_targ, motor_state.speed_targ, &plan_pos_f, &plan_speed_f) == 0)	//连续规划
			{
				fml_radio_servo_pos_cmd(plan_pos_f, plan_speed_f, 1);
			}
			else
			{
				motor_state.run_state = E_STATE_NONE;
				motor_state.bak_state = E_STATE_STOP_NONE;
				g_dio_plan_state = 0;
			}
		}
		else if(motor_state.run_state == E_STATE_STOP) //轨迹急停
		{
			if(otg_update(motor_state.postion_targ, motor_state.speed_targ, &plan_pos_f, &plan_speed_f) == 0)
			{
				fml_radio_servo_pos_cmd(plan_pos_f, plan_speed_f, 1);
			}
			else
			{
				motor_state.run_state = E_STATE_NONE;
				motor_state.bak_state = E_STATE_STOP_NONE;
				g_dio_plan_state = 0;
			}
		}
		fml_current_filter(); //电流滤波
	}
}

//can数据接收任务
void fdcan_recv_task_entry(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pData)
{
	s_queue_can_data recv_data = {0};
	if(!FullQueue(&qFdcan1Recv))
	{
		recv_data.id = pHeader->Identifier;
		recv_data.len = FDCANLEN_TO_LEN(pHeader->DataLength);
		memcpy(recv_data.data, pData, recv_data.len);
		Enqueue(&qFdcan1Recv, &recv_data);
	}
}

//串口数据发送任务
void uart_send_task_entry(void)
{
	static uint32_t current_time = 0;
	
	if(HAL_GetTick() > current_time + 1) //1ms
	{
		current_time = HAL_GetTick();
		s_queue_uart_data send_data = {0};
		if(!EmptyQueue(&qUart1Send))
		{
			Dequeue(&qUart1Send, &send_data);
			if(send_data.len > 0 && send_data.len < 65) 
			{
				hal_uart1_send(send_data.data, send_data.len);
			}
		}
		__hal_iwdg_reload(); // 看门狗复位
	}
}

//串口数据接收任务
void uart_recv_task_entry(uint8_t *pData, uint16_t len)
{
	s_queue_uart_data recv_data = {0};	
	if(!FullQueue(&qUart1Recv))
	{
		recv_data.len = len;
		memcpy(recv_data.data, pData, recv_data.len);
		Enqueue(&qUart1Recv, &recv_data);
	}
}

//can数据解析任务
void fdcan_data_parse_task_entry(void)
{
	static uint32_t current_time = 0;
	
	if(HAL_GetTick() > current_time + 1) //1ms
	{
		current_time = HAL_GetTick();
		s_queue_can_data recv_data = {0};
		if(!EmptyQueue(&qFdcan1Recv))
		{
			Dequeue(&qFdcan1Recv, &recv_data);
			fml_radio_data_parse(&recv_data); //广播帧解析
			//fml_fdcan_data_parse(&recv_data); //普通帧解析
		}
	}
}

//串口数据解析任务
void uart_data_parse_task_entry(void)
{
	static uint32_t current_time = 0;
	
	if(HAL_GetTick() > current_time + 1) //1ms
	{
		current_time = HAL_GetTick();
		uint8_t resp_buf[100] = {0};
		s_queue_uart_data recv_data = {0};
		if(!EmptyQueue(&qUart1Recv))
		{
			Dequeue(&qUart1Recv, &recv_data);
			if(recv_data.len > 0) 
			{
				if(recv_data.data[0] == 0xEB && recv_data.data[1] == 0x90) //因时夹爪协议
				{
					apl_485_protocol_parase(recv_data.data, recv_data.len);
				} 
				else //标准modbus协议 
				{ 
					int ind = fml_mb_slave_process(recv_data.data, recv_data.len, resp_buf); //modbus请求
					if(ind > 0) //modbus帧解析正确
					{
						hal_uart1_send(resp_buf, ind); //modbus响应
					}
				}
			}
		}
	}
}

//力位控制
void force_postion_task_entry(void)
{
	static uint8_t count = 0;
	static uint32_t current_time = 0;
	
	if(HAL_GetTick() > current_time + 10) //10ms
	{
		current_time = HAL_GetTick();
		//力位控制
		if(motor_state.force_enable == RM_TRUE)
		{
			if(motor_state.cmd_state == E_STATE_FORCE_ONCE) //单次夹取
			{
				if(motor_state.force > motor_state.force_range)
				{
					fml_radio_servo_pos_force_cmd(motor_state.postion + 0.01, 0, 0); //张开
				}
				else if(motor_state.force > RM_FORCE_NONE && motor_state.force < motor_state.force_range)
				{
					fml_radio_servo_pos_force_cmd(motor_state.postion - 0.01, 0, 0); //闭合
				}
				else if(motor_state.force <= RM_FORCE_NONE) //已经松开
				{
					motor_state.force_enable = RM_FALSE; //关闭力控
					update_motor_real_position(motor_state.postion); //更新当前角度
				}
			}
			else if(motor_state.cmd_state == E_STATE_FORCE_KEEP) //持续夹取
			{
				if(motor_state.force > motor_state.force_range)
				{
					fml_radio_servo_pos_force_cmd(motor_state.postion + 0.01, 0, 0); //张开
				}
				else if(motor_state.force > RM_FORCE_NONE && motor_state.force < motor_state.force_range)
				{
					fml_radio_servo_pos_force_cmd(motor_state.postion - 0.01, 0, 0); //闭合
				}
				else if(motor_state.force < RM_FORCE_NONE) //已经松开
				{
					//规划到最小位置
					motor_state.force_enable = RM_FALSE; //关闭力控
					update_motor_real_position(motor_state.postion); //更新当前角度
					motor_state.cmd_state = E_STATE_FORCE_KEEP; //持续夹取
					cmd_servo_plan_init(SYSTEM_PARAM()->servo_speed, SYSTEM_PARAM()->servo_force, SYSTEM_PARAM()->pos_limit_min, E_STATE_PLAN_CLOSE); //继续规划
				}
			}
			
			//更新算法当前角度
			if(motor_state.postion - plan_pos_f > 2)
			{
				//LOG_D("update:%.2f %.2f\r\n", motor_state.postion, plan_pos_f);
				update_motor_real_position(motor_state.postion); //更新当前角度
			}
		}
		
		if(++count > 10) //100ms
		{
			count = 0;
			//更新MB IO相关寄存器
			fml_modbus_reg_update(); 
			
			//IO控制夹爪轮训
			if(spt.key_enable)
			{
				dio_plan_sub_task();
			}
		}
	}
}

//IO 控制
void dio_plan_sub_task(void)
{
	//松开夹爪
	if(g_dio_state[0].pin_mode == E_DIO_MODE_IN)
	{
		if(g_dio_state[0].pin_state == E_DIO_LOW && g_dio_plan_state == 0) //低电平触发
		{
			if(motor_state.open_range < RM_POS_MAX)
			{
				g_dio_plan_state = 1; //松开使能
				uint16_t speed = SYSTEM_PARAM()->servo_speed;
				uint16_t force = SYSTEM_PARAM()->servo_force;
				uint16_t postion = SYSTEM_PARAM()->pos_limit_max;
				cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_OPEN); //运动规划
			}
		}
		else if(g_dio_state[0].pin_state == E_DIO_HIGH && g_dio_plan_state == 1) //高电平急停
		{
			cmd_servo_stop_init(); //急停规划
		}
	}
	
	//闭合夹取
	if(g_dio_state[1].pin_mode == E_DIO_MODE_IN)
	{
		if(g_dio_state[1].pin_state == E_DIO_LOW && g_dio_plan_state == 0) //低电平触发
		{
			if(motor_state.open_range > RM_POS_MIN)
			{
				g_dio_plan_state = 2; //松开使能
				uint16_t speed = SYSTEM_PARAM()->servo_speed;
				uint16_t force = SYSTEM_PARAM()->servo_force;
				uint16_t postion = SYSTEM_PARAM()->pos_limit_min;
				motor_state.cmd_state = E_STATE_FORCE_ONCE; //单次夹取
				cmd_servo_plan_init(speed, force, postion, E_STATE_PLAN_CLOSE); //运动规划
			}
		}
		else if(g_dio_state[1].pin_state == E_DIO_HIGH && g_dio_plan_state == 2)  //高电平急停
		{
			cmd_servo_stop_init(); //急停规划
		}
	}
}

//电机实时状态跟新
void state_update_task_entry(void)
{
	static uint8_t init_step = 4;
	static uint32_t current_time = 0;
	static uint8_t block_count = 0;
	
	if(init_step <= 0)
	{
		if(HAL_GetTick() > current_time + 500) //100ms
		{
			current_time = HAL_GetTick();
			if(motor_state.run_state == E_STATE_NONE)
			{
				fml_radio_servo_state_cmd(); //伺服状态查询
			}
			//if(motor_state.pre_state == E_STATE_PLAN_OPEN) //开启是遇到堵转
			{
				//堵转处理-暂时处理张开时的堵转-后续开发外夹指重做
				if(motor_state.current > 4000) 
				{
					block_count++;
					if(block_count > 20) //10s持续高电流表示堵转
					{
						fml_radio_servo_pos_none_cmd(motor_state.postion - 1, 0, 0); //设置当前位置为目标位置
						update_motor_real_position(motor_state.postion); //更新算法当前角度
						block_count = 0;
					}
				} 
				else 
				{
					block_count = 0;
				}
			}
			
			motor_state.timeout++; //电机通讯超时
			if((motor_state.error&E_ERR_VOER_TMP || motor_state.temp > RM_TEMP_MAX)&& motor_state.timeout > 10) //过温且通讯异常则电机堵转
			{
				motor_state.sys_error = motor_state.sys_error |(0x01<<0);
			}
			else if(motor_state.error & E_ERR_VOER_TMP || motor_state.temp > RM_TEMP_MAX) //过温
			{
				motor_state.sys_error = motor_state.sys_error |(0x01<<1);
			}
			else if(motor_state.error&E_ERR_OVER_CUR) //过流
			{
				motor_state.sys_error = motor_state.sys_error |(0x01<<2); 
			}
			else if(motor_state.error&E_ERR_FOC) //驱动器故障
			{
				motor_state.sys_error = motor_state.sys_error|(0x01<<3);
			}
			else if(motor_state.timeout > 10) //内部通讯故障
			{
				motor_state.sys_error = motor_state.sys_error|(0x01<<4);
			} 
			else if(motor_state.error != 0) //电机错误
			{
				motor_state.sys_error = motor_state.sys_error|(0x01<<5);
			}
			else			
			{
				motor_state.sys_error = 0;
			}
			// LED错误状态
			if(motor_state.error != 0 || motor_state.sys_error != 0)
			{
				fml_set_rgb(E_RGB_RED);
			}
			else
			{
				fml_set_rgb(E_RGB_GREEN);
			}
			
			//升级等待重启超时
			if(g_upgrade_enable > 0) 
			{
				if(g_upgrade_enable <= 1)
				{
					apl_upgrade_timeout_handle();
				}
				g_upgrade_enable--;
			}	

			//算法初始化
			if(motor_state.enable == 1)
			{
				if(algo_init_flag == 0)
					fml_algo_init();
			}
		}
	}
	else
	{
		if(HAL_GetTick() > current_time + 100) //100ms
		{
			current_time = HAL_GetTick();
			
			if(init_step == 4)
			{
				fml_radio_servo_reg_read(0x01, 1); //读取ID
			}
			else if(init_step == 3)
			{
				//fml_radio_servo_reg_write(0x39, 0, 3); //设置高跟随 1-低跟随 0-高跟随 默认高跟随
			}
			else if(init_step == 2)
			{
				fml_radio_servo_reg_write(0x0A, 1, 4); //关节上使能
			}
			else if(init_step == 1)
			{
				fml_radio_servo_reg_write(0x30, POS_MODE, 4); //伺服模式-位置环
			}
			init_step--;
		}
	}
}

void fml_dio_init(void)
{
	if(spt.DIO1_mode == E_DIO_MODE_IN) //输入模式
	{
		DOUT1(0);
		g_dio_state[0].pin_mode = E_DIO_MODE_IN;
	}
	else //输出模式 
	{
		DOUT1(1);
		g_dio_state[0].pin_mode = E_DIO_MODE_OUT;
		
		if(spt.DIO1_state == 0)
		{
			DOUT1(1);
		}
		else
		{
			DOUT1(0);
		}
	}

	if(spt.DIO2_mode == E_DIO_MODE_IN) //输入模式
	{
		DOUT2(0);
		g_dio_state[1].pin_mode = E_DIO_MODE_IN;
	}
	else //输出模式 
	{
		DOUT2(1);
		g_dio_state[1].pin_mode = E_DIO_MODE_OUT;
		
		if(spt.DIO1_state == 0)
		{
			DOUT2(1);
		}
		else
		{
			DOUT2(0);
		}
	}
}
