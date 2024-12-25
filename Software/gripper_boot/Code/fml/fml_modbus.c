#include "fml_modbus.h"
#include "fml_storage.h"
#include "common.h"

s_mb_rtu_cb *pWriteRegCb = NULL;

static unsigned int MB_COIL_NUM    = S_COIL_NCOILS; //线圈数量
static unsigned int MB_DI_NUM      = S_DISCRETE_INPUT_NDISCRETES; //离散变量
static unsigned int MB_HOLDING_NUM = S_REG_HOLDING_NREGS; //保持寄存器
static unsigned int MB_INPUT_NUM   = S_REG_INPUT_NREGS; //输入寄存器
  
static unsigned char DEV_ADDR = 0x01; //从机地址

////离散输入
//#if S_DISCRETE_INPUT_NDISCRETES%8
//unsigned char ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
//#else
//unsigned char ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8];
//#endif
////线圈
//#if S_COIL_NCOILS%8
//unsigned char ucSCoilBuf[S_COIL_NCOILS/8+1];
//#else
//unsigned char ucSCoilBuf[S_COIL_NCOILS/8];
//#endif
//离散输入
unsigned char ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES];
//线圈
unsigned char ucSCoilBuf[S_COIL_NCOILS];
//输入寄存器
unsigned short usSRegInBuf[S_REG_INPUT_NREGS];
//保持寄存器
unsigned short usSRegHoldBuf[S_REG_HOLDING_NREGS];

/* 高位字节的 CRC 值 */
static unsigned char auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };
/* 低位字节的 CRC 值 */
static char auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8,
        0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5,
        0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33,
        0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E,
        0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3,
        0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5,
        0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8,
        0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4,
        0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F,
        0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A,
        0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82,
        0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };
 
static unsigned short CRC16_ModBus(unsigned char *puchMsg, unsigned char usDataLen)
{
    unsigned char uchCRCHi = 0xFF; /* CRC 的高字节初始化 */
    unsigned char uchCRCLo = 0xFF; /* CRC 的低字节初始化 */
    unsigned char uIndex; /* CRC 查询表索引 */
    while (usDataLen--) /* 完成整个报文缓冲区 */
    {
        uIndex = uchCRCLo ^ *puchMsg++; /* 计算 CRC */
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}
 
static int frame_available(unsigned char *buf, int len)
{
    int data_len = 0, num;
    if (len < 5)
        return 0;
 
    if (buf[1] == 1 || buf[1] == 2 || buf[1] == 3 || buf[1] == 4 || buf[1] == 5 || buf[1] == 6)
    {
        data_len = 4;
    }
    else if (buf[1] == 15)
    {
        num = ((buf[4] << 8) | buf[5]);
        data_len = 4 + 1 + (num + 7) / 8;
    }
    else if (buf[1] == 16)
    {
        num = ((buf[4] << 8) | buf[5]);
        data_len = 4 + 1 + num * 2;
    }
    else if (buf[1] & 0x80)
    {
        data_len = 1;
    }
    else
    {
        return -1;
    }
 
    if (len < data_len + 4)
        return 0;
 
    unsigned short crc = CRC16_ModBus(buf, 2 + data_len);
    if (crc == (buf[2 + data_len] | ((buf[2 + data_len + 1]) << 8)))
    {
        return 1;
    }
 
    return -1;
}

//写线圈
static int mb_write_coil_cb(unsigned short addr, unsigned short val)
{
	ucSCoilBuf[addr] = val;
	return 0;
}

//写保持寄存器
static int mb_write_holding_cb(unsigned short addr, unsigned short val)
{
	usSRegHoldBuf[addr] = val;
	return 0;
}

//读线圈
static unsigned short mb_read_coil_cb(unsigned short addr)
{
	return ucSCoilBuf[addr];
}

//读离散输入
static unsigned short mb_read_di_cb(unsigned short addr)
{
	return ucSDiscInBuf[addr];
}

//读保持寄存器
static unsigned short mb_read_holding_cb(unsigned short addr)
{
	return usSRegHoldBuf[addr];
}

//读输入寄存器
static unsigned short mb_read_input_cb(unsigned short addr)
{
	return usSRegInBuf[addr];
}

//mb初始化
void fml_mb_slave_init(unsigned char slave_addr, int coil_num, int di_num, int holding_num, int input_num)
{
    MB_COIL_NUM    = coil_num;
    MB_DI_NUM      = di_num;
    MB_HOLDING_NUM = holding_num;
    MB_INPUT_NUM   = input_num;
 
    DEV_ADDR = slave_addr;
		//数据存储
}

//数据解析
int fml_mb_slave_process(unsigned char *rx_buf, int rx_len, unsigned char *tx_buf)
{
    unsigned char error_code = 0;
    unsigned short num, addr, val, ind = 0, crc16, bytes;
    int ret = frame_available(rx_buf, rx_len);
 
    //frame error
    if (ret <= 0)
        return ret;
 
    //valid addr
    if (rx_buf[0] != DEV_ADDR)
        return 0;
 
    //process rtu cmd
    switch (rx_buf[1])
    {
    case 1: //read coil
    case 2: //read di
        addr = (rx_buf[2] << 8) | rx_buf[3];
        num = (rx_buf[4] << 8) | rx_buf[5];
        if (num < 1 || num > 0x7d0)
        {
            error_code = 3;
            break;
        }
        if (addr + num > (rx_buf[1] == 1 ? MB_COIL_NUM : MB_DI_NUM))
        {
            error_code = 2;
            break;
        }
        tx_buf[ind++] = DEV_ADDR;
        tx_buf[ind++] = rx_buf[1];
        bytes = (num + 7) / 8;
        tx_buf[ind++] = bytes;
        for (int i = 0; i < bytes; i++)
        {
            tx_buf[ind + i] = 0;
        }
        for (int i = 0; i < num; i++)
        {
            if (rx_buf[1] == 1)//coil
            {
                val = mb_read_coil_cb(i + addr);
                if (val)
                    tx_buf[ind + i / 8] |= 1 << (i % 8);
            }
            else//di
            {
                val = mb_read_di_cb(i + addr);
                if (val)
                    tx_buf[ind + i / 8] |= 1 << (i % 8);
            }
        }
        ind += bytes;
        crc16 = CRC16_ModBus(tx_buf, ind);
        tx_buf[ind++] = crc16;
        tx_buf[ind++] = crc16 >> 8;
        break;
 
    case 3: //read holding
    case 4: //read input
        addr = (rx_buf[2] << 8) | rx_buf[3];
        num = (rx_buf[4] << 8) | rx_buf[5];
        if (num < 1 || num > 125)
        {
            error_code = 3;
            break;
        }
        if (addr + num > (rx_buf[1] == 3 ? MB_HOLDING_NUM : MB_INPUT_NUM))
        {
            error_code = 2;
            break;
        }
        tx_buf[ind++] = DEV_ADDR;
        tx_buf[ind++] = rx_buf[1];
        tx_buf[ind++] = num * 2;
        for (int i = 0; i < num; i++)
        {
            if (rx_buf[1] == 3)//holding
            {
                val = mb_read_holding_cb(i + addr);
                tx_buf[ind++] = val >> 8;
                tx_buf[ind++] = val;
            }
            else//input
            {
                val = mb_read_input_cb(i + addr);
                tx_buf[ind++] = val >> 8;
                tx_buf[ind++] = val;
            }
        }
        crc16 = CRC16_ModBus(tx_buf, ind);
        tx_buf[ind++] = crc16;
        tx_buf[ind++] = crc16 >> 8;
        break;
 
    case 5: //write single coil
    case 6: //write single reg
        addr = (rx_buf[2] << 8) | rx_buf[3];
        val = (rx_buf[4] << 8) | rx_buf[5];
        if (val != 0x0 && val != 0xff00 && rx_buf[1] == 5)
        {
            error_code = 3;
            break;
        }
        if (addr> (rx_buf[1] == 5 ? MB_COIL_NUM : MB_HOLDING_NUM))
        {
            error_code = 2;
            break;
        }
        for (int i = 0; i < 8; i++)
        {
            tx_buf[i] = rx_buf[i];
        }
        if (rx_buf[1] == 5)
        {
            if (0 != mb_write_coil_cb(addr, val))
                error_code = 4;
//						if(pWriteRegCb != NULL && error_code == 0) 
//						{
//							pWriteRegCb->wirte_coils_cb(rx_buf, rx_len); //写线圈处理函数
//						}
        }
        else
        {
#if 0
					if (0 != mb_write_holding_cb(addr, val))
						error_code = 4;
#else					
					if(pWriteRegCb != NULL && error_code == 0) 
					{
						if(pWriteRegCb->wirte_holding_cb(rx_buf, rx_len) == RM_OK) //写保持寄存器处理函数
						{
							if (0 != mb_write_holding_cb(addr, val))
								error_code = 4;
						}
					}
					else
					{
						error_code = 4;
					}
#endif
        }
        ind = 8;
        break;
 
    case 15: //wrtie multi coils
        addr = (rx_buf[2] << 8) | rx_buf[3];
        num = (rx_buf[4] << 8) | rx_buf[5];
        if (num < 1 || num > 0x7b || rx_buf[6] != (num + 7) / 8)
        {
            error_code = 3;
            break;
        }
        if (addr + num > MB_COIL_NUM)
        {
            error_code = 2;
            break;
        }
        for (ind = 0; ind < 6; ind++)
        {
            tx_buf[ind] = rx_buf[ind];
        }
        for (int i = 0; i < num; i++)
        {
            val = rx_buf[7 + i / 8] & (1 << (i % 8));
            if (mb_write_coil_cb(addr + i, val) != 0)
                error_code = 4;
        }
//				if(pWriteRegCb != NULL && error_code == 0) 
//				{
//					pWriteRegCb->wirte_coils_cb(rx_buf, rx_len); //写线圈处理函数
//				}
        crc16 = CRC16_ModBus(tx_buf, ind);
        tx_buf[ind++] = crc16;
        tx_buf[ind++] = crc16 >> 8;
        break;
 
    case 16: //write multi regs
        addr = (rx_buf[2] << 8) | rx_buf[3];
        num = (rx_buf[4] << 8) | rx_buf[5];
        if (num < 1 || num > 0x7b || rx_buf[6] != num * 2)
        {
            error_code = 3;
            break;
        }
        if (addr + num > MB_HOLDING_NUM)
        {
            error_code = 2;
            break;
        }
        for (ind = 0; ind < 6; ind++)
        {
            tx_buf[ind] = rx_buf[ind];
        }
#if 0
        for (int i = 0; i < num; i++)
        {
            val = (rx_buf[7 + 2 * i] << 8) | rx_buf[7 + 2 * i + 1];
            if (0 != mb_write_holding_cb(addr + i, val))
                error_code = 4;
        }
				if(pWriteRegCb != NULL && error_code == 0) 
				{
					pWriteRegCb->wirte_holding_cb(rx_buf, rx_len); //写保持寄存器处理函数
				}
#else
				if(pWriteRegCb != NULL && error_code == 0) 
				{
					if(pWriteRegCb->wirte_holding_cb(rx_buf, rx_len) == RM_OK) //写保持寄存器处理函数
					{
						for (int i = 0; i < num; i++)
						{
								val = (rx_buf[7 + 2 * i] << 8) | rx_buf[7 + 2 * i + 1];
								if (0 != mb_write_holding_cb(addr + i, val))
										error_code = 4;
						}
					}
				}
				else
				{
					error_code = 4;
				}
#endif				
        crc16 = CRC16_ModBus(tx_buf, ind);
        tx_buf[ind++] = crc16;
        tx_buf[ind++] = crc16 >> 8;
        break;
    default:
        if (rx_buf[1] & 0x80)
            return 0;
        else
            error_code = 1;
        break;
    }
    if (error_code != 0)
    {
        ind = 0;
        tx_buf[ind++] = DEV_ADDR;
        tx_buf[ind++] = rx_buf[1] | 0x80;
        tx_buf[ind++] = error_code;
        crc16 = CRC16_ModBus(tx_buf, ind);
        tx_buf[ind++] = crc16;
        tx_buf[ind++] = crc16 >> 8;
    }
    return ind;
}

void fml_modbus_update_reg(e_reg_type reg_type, uint16_t addr, uint16_t value)
{
	if(reg_type == E_REG_COILS)
	{
		 ucSCoilBuf[addr] = (uint8_t)value;
	}
	else if(reg_type == E_REG_DISCRETE_INPUT)
	{
		ucSDiscInBuf[addr] = (uint8_t)value;
	}
	else if(reg_type == E_REG_HOLDING)
	{
		usSRegHoldBuf[addr] = (uint16_t)value;
	}
	else if(reg_type == E_REG_INPUT)
	{
		usSRegInBuf[addr] = (uint16_t)value;
	}
}

void fml_modbus_param_init(s_mb_rtu_cb *_cb)
{
	DEV_ADDR = SYSTEM_PARAM()->device_id;
	pWriteRegCb = _cb;
	
	//寄存器初始化
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_DEV_ID, DEV_ADDR); //设备ID
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_UART_BAUD, SYSTEM_PARAM()->uart1_baud); //波特率
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SET_SPEED, SYSTEM_PARAM()->servo_speed); //速度
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_SET_FORCE, SYSTEM_PARAM()->servo_force); //力度
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_MAX, SYSTEM_PARAM()->pos_limit_max); //最大限位
	fml_modbus_update_reg(E_REG_HOLDING, MB_REG_POS_MIN, SYSTEM_PARAM()->pos_limit_min); //最小限位
}

