#ifndef __COMMON_H
#define __COMMON_H

#include "stm32g4xx_hal.h"

typedef _Bool						uint1;
typedef unsigned char   uint8;
typedef char						int8;
typedef unsigned short  uint16;
typedef short						int16;
typedef unsigned int    uint32;
typedef int							int32;
typedef unsigned int		size_t;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define TRUE          (1)
#define FALSE         (0)

//#define DBG_ENABLE

#ifndef VERSION
#define VERSION "189"
#endif

#define BREAK_UINT32( var, ByteNum ) \
          (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32)((uint32)((Byte0) & 0x00FF) \
          + ((uint32)((Byte1) & 0x00FF) << 8) \
          + ((uint32)((Byte2) & 0x00FF) << 16) \
          + ((uint32)((Byte3) & 0x00FF) << 24)))
					
#define BUILD_INT32(Byte0, Byte1, Byte2, Byte3) \
          ((int)((uint32)((Byte0) & 0x00FF) \
          + ((int)((Byte1) & 0x00FF) << 8) \
          + ((int)((Byte2) & 0x00FF) << 16) \
          + ((int)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define BUILD_INT16(loByte, hiByte) \
          ((short)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))					
					
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

#define BUILD_UINT8(hiByte, loByte) \
          ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

typedef enum {
	RM_OK,
	RM_ERROR,
}rm_result_e;

#define RM_TRUE            (1)
#define RM_FALSE					 (0)

#define EF                  1 //力转电流系数 = 10N/1200mA 
#define RM_CURRENT_MAX      10000  //电流做大值 = 额定电流(800mA)x3 mA
#define RM_FORCE_MIN     		(1) //力最小值
#define RM_FORCE_MAX     		(1000) //力做大值
#define RM_SPEED_MIN     		(1) //速度最小值 1%
#define RM_SPEED_MAX     		(1000) //速度最大值 100%
#define RM_POS_MIN     	 		(0) //位置最小值 0mm
#define RM_POS_MAX       		(1000) //位置最大值 65mm
#define RM_RANGE_MIN     	 	(0) //位置最小值 0mm
#define RM_RANGE_MAX       	(65.0) //位置最大值 65mm
#define RM_ANGLE_MIN     	 	(0) //角度最小值
#define RM_ANGLE_MAX       	(87.0) //角度最大值

#define RM_HIGHT_FORCE_ENABLE //开启大力度功能

#ifdef RM_HIGHT_FORCE_ENABLE
#define RM_CURRENT_OFFSET  	(3000) //克服静摩擦电流
#define RM_FORCE_CTRL_MIN  	(500) //力控最小值
#define RM_FORCE_NONE     	(400) //夹持空载阈值-小于此值默认此时没有夹持任何东西
#else
#define RM_CURRENT_OFFSET  	(800) //克服静摩擦电流
#define RM_FORCE_CTRL_MIN  	(400) //力控最小值
#define RM_FORCE_NONE     	(300) //夹持空载阈值-小于此值默认此时没有夹持任何东西
#endif

#define RM_TEMP_MAX       	(85)

void log_printf(const char * msg, ...);

#endif /* __CTRL */
