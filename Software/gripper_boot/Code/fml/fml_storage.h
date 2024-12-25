#ifndef __FML_STORAGE_H
#define __FML_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "common.h"

#define STORAGE_BASE_ADDR 0x0801F000

typedef enum {
	E_DATA_ALL							= 0x00,
	E_DEVICE_ID										,
	E_UART_BAUD										,
}e_sys_parm_items;

typedef enum {
	READ_FLASH,
	DO_NOT_READ_FLASH
}e_apt_flag;


#pragma pack(1)
typedef struct {
/*
	system
*/
	__IO uint8_t valid;
	__IO uint8_t device_id; //modbus slave id
	__IO uint8_t uart1_baud; //uart baud
	__IO uint8_t upgrade_en; //0-不升级 1-控制板升级 2-关节升级
/*
	servo
*/
	__IO uint16_t pos_limit_min; //最小限位 0-65 对应电机角度0-60度
	__IO uint16_t pos_limit_max; //最大限位 0-65 对应电机角度0-60度
	
	__IO uint16_t servo_speed;   //运动速度0-100
	__IO uint16_t servo_force;  //夹取力度
	
	__IO uint8_t motor_id; //motor id
	__IO uint8_t key_enable; //按键控制使能
}system_parameters_t;
#pragma pack()

extern system_parameters_t spt;

void fml_system_param_init(void);
system_parameters_t *fml_storage_get(e_apt_flag flag);
rm_result_e fml_storage_save(uint32_t items);
void fml_system_param_default(void);
system_parameters_t *SYSTEM_PARAM(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
