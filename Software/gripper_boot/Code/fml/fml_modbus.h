#ifndef __FML_MODBUS_H
#define __FML_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#define S_DISCRETE_INPUT_START                    0
#define S_DISCRETE_INPUT_NDISCRETES               10
#define S_COIL_START                              0
#define S_COIL_NCOILS                             10
#define S_REG_INPUT_START                         0
#define S_REG_INPUT_NREGS                         10
#define S_REG_HOLDING_START                       0
#define S_REG_HOLDING_NREGS                       128
	
typedef enum
{
	E_REG_COILS 							= 0x01,
	E_REG_DISCRETE_INPUT			,
	E_REG_HOLDING 						,
	E_REG_INPUT 							,
}e_reg_type;
	
typedef enum
{
	E_READ_COILS 							= 0x01, /*读线圈状态*/
	E_READ_DISCRETE_INPUT 		= 0x02, /*读离散输入状态*/
	E_READ_HOLDING 						= 0x03, /*读保持寄存器*/
	E_READ_INPUT 							= 0x04, /*读输入寄存器*/
	E_WRITE_SINGLE_COIL 			= 0x05, /*写单个线圈*/
	E_WRITE_SINGLE_HOLDING 		= 0x06, /*写单个保持寄存器*/
	E_WRITE_COILS 						= 0x0f, /*写多个线圈*/
	E_WRITE_HOLDINGS 					= 0x10, /*写多个保持寄存器*/	
}e_mb_func_code;

typedef enum
{
	MB_ENOERR = 0,              /*!< no error. */
	MB_ENOREG,                  /*!< illegal register address. */
	MB_EINVAL,                  /*!< illegal argument. */
	MB_EPORTERR,                /*!< porting layer error. */
	MB_ENORES,                  /*!< insufficient resources. */
	MB_EIO,                     /*!< I/O error. */
	MB_EILLSTATE,               /*!< protocol stack in illegal state. */
	MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;

//设备信息地址表-保持寄存器
typedef enum /**/
{
	MB_REG_PARAM_SAVE 	 = 0x0001, /*保存参数 (rw)*/
	MB_REG_PARAM_DEFAULT 				 , /*参数恢复默认 (rw)*/
	MB_REG_DEV_ID 							 , /*设置设备ID (rw)*/
	MB_REG_UART_BAUD 						 , /*设置串口波特率 (rw)*/
	
	MB_REG_CATCH 								 , /*力控夹取 (rw)*/
	MB_REG_STOP 								 , /*急停 (rw)*/
	MB_REG_ERR_CLE 							 , /*清除错误 (rw)*/
	MB_REG_SET_POS 							 , /*设置夹爪开口度 (rw)*/
	MB_REG_SET_SPEED 						 , /*设置夹取速度 (rw)*/
	MB_REG_SET_FORCE 						 , /*设置夹取力度 (rw)*/
	MB_REG_POS_MAX 							 , /*设置最大开口度 (rw)*/
	MB_REG_POS_MIN 							 , /*设置最小开口度 (rw)*/
	MB_REG_FORCE_REAL 					 , /*实际力度 (ro)*/
	MB_REG_POS_REAL 						 , /*实际位置 (ro)*/
	MB_REG_CURRENT_REAL 				 , /*实际电流 (ro)*/
	MB_REG_VOLTAGE_REAL 				 , /*实际电压 (ro)*/
	MB_REG_TEMP_REAL 						 , /*实际温度 (ro)*/
	MB_REG_ERROR_CODE 					 , /*夹爪错误码 (ro)*/
	MB_REG_SERVO_STATE 					 , /*夹爪状态码 (ro)*/
} e_reg_info_map;

typedef struct {
	uint8_t (*wirte_coils_cb)(uint8_t * data, uint16_t len);
	uint8_t (*wirte_holding_cb)(uint8_t * data, uint16_t len);
}s_mb_rtu_cb;

void fml_modbus_param_init(s_mb_rtu_cb *_cb);
void fml_mb_slave_init(unsigned char slave_addr, int coil_num, int di_num, int holding_num, int input_num);
int fml_mb_slave_process(unsigned char *rx_buf, int rx_len, unsigned char *tx_buf);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL */
