#include "IAP.h"

iapfun jump2app; 

//__asm void MSR_MSP(uint32_t addr) 
//{
//    MSR MSP, r0 			//set Main Stack value
//    BX r14
//}

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(uint32_t appxaddr)
{ 
//	if(((*(volatile uint32_t*)appxaddr)&0x2FF00000)==0x24000000)	//检查栈顶地址是否合法.  // 目前没怎么用，暂时合适
	{ 
		jump2app=(iapfun)*(volatile uint32_t*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
//		MSR_MSP(*(volatile uint32_t*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
//		__set_PSP(*(volatile uint32_t*) appxaddr);

 		__set_MSP(*(volatile uint32_t*) appxaddr);
		jump2app();									//跳转到APP.
	}
}		 


