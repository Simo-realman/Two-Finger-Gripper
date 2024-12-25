#include "hal_flash.h"
#include "string.h"

rm_result_e hal_flash_read_buf(uint32_t addr, uint8_t *buf, uint32_t len)
{
	if(addr < FLASH_BASE_ADDR || addr%4)   
		return RM_ERROR;
	
#if 1	
	__IO uint8_t *index = (__IO uint8_t *)(addr);
	while(len--)
	{
		*buf++=*index++;
	}
#else
	for(uint32_t i=0; i < len; i++)
	{
		buf[i]=*(uint8_t *)(addr+i);
	}
#endif
	return RM_OK;
}

rm_result_e hal_flash_write_buf(uint32_t addr, uint8_t *buf, uint32_t len)
{
	#define MAX_SAVE_SIZE 10

	if (addr < FLASH_BASE_ADDR || addr % 4 || len > MAX_SAVE_SIZE * 8)
			return RM_ERROR;

	uint32_t size = (len + 7) / 8;  // 计算需要的64位块数量
	uint64_t databuf[MAX_SAVE_SIZE] = {0};

	memcpy(databuf, buf, len);

	while(FLASH_WaitForLastOperation(50000)!=HAL_OK);
	
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	if (FLASH_WaitForLastOperation(50000) == HAL_OK) {
			for (uint32_t i = 0, flash_addr = addr; i < size; i++, flash_addr += 8) {
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_addr, databuf[i]) != HAL_OK)
							break;
					HAL_Delay(1);
					//LOG_D("write: %d %d\r\n", flash_addr, addr + size * 8);
			}
	}
	HAL_FLASH_Lock();
	
	return RM_OK;
}

void hal_flash_erase(uint8_t page, uint32_t appsize)
{
	FLASH_EraseInitTypeDef FlashEraseInit;
	uint8_t Erase_page_num=0;
	uint8_t i=0;
	uint32_t PageError;
			
	while(FLASH_WaitForLastOperation(50000)!=HAL_OK);
	if((appsize%(2*1024))==0)
		Erase_page_num=appsize/(2*1024);
	else
		Erase_page_num=appsize/(2*1024)+1;
	
	for(i=0;i<Erase_page_num;i++)
	{				
		HAL_FLASH_Unlock();
	
		FlashEraseInit.Banks=FLASH_BANK_1;//FLASH_BANK_1;
		FlashEraseInit.Page=page;//FLASH_Page63;
		FlashEraseInit.NbPages=1;		
		FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;
		while(HAL_FLASHEx_Erase(&FlashEraseInit,&PageError)!=HAL_OK);
		
		HAL_FLASH_Lock();
	}		
}


