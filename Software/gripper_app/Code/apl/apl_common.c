#include "common.h"
#include "stdarg.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "hal_usart.h"

#define LOG_BUF_SIZE 512

void log_printf(const char * msg, ...)
{
    uint16 len = 0;
	
    char log_buf[LOG_BUF_SIZE] = {0};
    va_list args;

    if(msg==NULL) return;

    va_start(args, msg);
		
    vsprintf(log_buf, (const char*)msg, args);
		
		len=strlen((char *)log_buf);
		if(len > LOG_BUF_SIZE-1) len = LOG_BUF_SIZE-1;
		hal_uart3_send((uint8_t *)log_buf, len);
		
    va_end(args);
}


uint16_t get_build_version(void)
{
	uint16_t version = 0;
	int temp = 0;
	char *pVersion = NULL;
	pVersion = VERSION;
	if(strlen(pVersion) == 3)
	{
			temp = ((pVersion[0] - '0') << 8) + ((pVersion[1] - '0') << 4) + ((pVersion[2] - '0'));
	}
	else if(strlen(pVersion) == 4)
	{
			temp = ((pVersion[0] - '0') << 12) + ((pVersion[1] - '0') << 8) + ((pVersion[2] - '0') << 4) + ((pVersion[3] - '0'));
	}
	else
	{
			return 0;
	}
	version = temp << 8 | temp >> 8;
	return version;
}

