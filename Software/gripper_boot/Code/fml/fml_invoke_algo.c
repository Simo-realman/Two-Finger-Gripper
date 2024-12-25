#include "fml_invoke_algo.h"
#include <stdlib.h>
#include "fml_storage.h"
#define DBG_TAG "algo."
#define DBG_LVL DBG_LOG
#include "apl_dbg.h"

#define ONE_STEP 0.5 //度
static uint32_t algo_period = 0;
static uint16_t plan_speed = 0, plan_direct = 0;
static int plan_pos_offset = 0;
static int plan_pos_start = 0;
static int pos_max = 0, pos_min = 0;

void fml_my_algo_init(int _pos_now, int _pos_tag, uint8_t _speed)
{
	plan_speed = (100 - _speed)*8; //比例放大
	
	if(plan_speed < 1)
		plan_speed = 1;
	
	plan_pos_start = _pos_now;
	plan_pos_offset = abs(_pos_tag - _pos_now);
	if(_pos_tag > _pos_now)
	{
		plan_direct = 1; //正向
	}
	else
	{
		plan_direct = 0; //反向
	}
	algo_period = 0;
	
	pos_min = (int)((float)SYSTEM_PARAM()->pos_limit_min/65.0*60); //位置转角度
	pos_max = (int)((float)SYSTEM_PARAM()->pos_limit_max/65.0*60); //位置转角度
	LOG_D("algo init:%d %d %d %d %d\r\n", plan_pos_start, plan_pos_offset, plan_speed,pos_min,pos_max);
}

uint8_t fml_my_algo_plan(int *pPos)
{
	algo_period++;
	
	if(plan_pos_offset > 0)
	{	
		if(algo_period%plan_speed == 0)
		{
			if(plan_direct == 1)
			{
				plan_pos_start += 1;
			}
			else
			{
				plan_pos_start -= 1;
			}
			plan_pos_offset--;
		}
		*pPos = plan_pos_start;		
		
		if(plan_pos_start > pos_max+1 || plan_pos_start < pos_min-1)
		{
			return 0;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}



