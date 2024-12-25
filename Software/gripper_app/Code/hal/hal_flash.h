#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"

#define FLASH_BASE_ADDR           0x08000000
#define FLASH_PAGE62_ADDR         0x0801F000
#define FLASH_PAGE_ADDR(n)        (FLASH_BASE_ADDR+2048*n)

rm_result_e hal_flash_read_buf(uint32_t addr, uint8_t *buf, uint32_t len);
rm_result_e hal_flash_write_buf(uint32_t addr, uint8_t *buf, uint32_t len);
void hal_flash_erase(uint8_t page, uint32_t appsize);

#endif





