#ifndef __IAP_H__
#define __IAP_H__
#include "main.h"

////////////////////////////////////////////////////////////////////////////////// 
extern int shengji_en;
typedef  void (*iapfun)(void);				//定义一个函数类型的参数.   
#define FLASH_APP1_ADDR		0x08020000  	//第一个应用程序起始地址(存放在FLASH)
											//保留0X08000000~0X0801FFFF的空间为Bootloader使用(共128KB)	   
void iap_load_app(uint32_t appxaddr);			//跳转到APP程序执行
#endif
