#ifndef CANTYPE_H
#define CANTYPE_H

//编码器cnt
//#define ENCODER_CNT    262144
#define ENCODER_CNT    3600000
#define ENCODER_CNT_L    10000

//电流前馈转换系系数
#define CURRENT_FEEDBACK_UNIT 0.5 //2mA  <-> mA
//目标速度和前馈速度转换系数
#define SPEED_CMD_UNIT (1000/12) //0.002RPM  <-> °/S

// 末端接口板ID
#define END_BROAD_ID 0xfe
// 末端接口板消息返回ID
#define END_BROAD_ID_R 0x1fe
// 末端接口板modbus指令发送ID
#define END_BROAD_MODBUS_ID 0x3fe
// 末端接口板周期查询ID
#define END_BROAD_LOOPQUERY_ID  0x5fe
// 末端接口板周期状态反馈ID
#define END_BROAD_LPQ_STATUS_ID 0x6fe
// 一维力系数ID
#define END_BROAD_ONEFORCE_RATIO_ID 0x44
// 一维力系数因子
#define END_BROAD_ONEFORCE_RATIO    1000000

//指令类型宏定义
#define CMD_RD			0x01		//读指令
#define CMD_WR			0x02		//写指令
#define CMD_WR_NR		0x03		//无应答写，节约带宽
#define CMD_WR_REG      0X04    //异步写
#define CMD_ACTION      0x05    //执行异步写
#define CMD_IAP         0x06    //IAP更新指令
#define CMD_RESET       0X11    //恢复出厂设置

#define SERVER_POSE_CTRL_RADIO_ID 0x2F //位置伺服广播ID
#define SERVER_SPEED_CTRL_RADIO_ID 0x3F //电流伺服广播ID
#define SERVER_CUR_CTRL_RADIO_ID 0x4F //电流伺服广播ID
#define SERVER_WR_CTRL_RADIO_ID 0x0F //读写广播ID
#define SERVER_STATE_RADIO_ID 0x7F //状态伺服广播ID

//CAN总线协议类型 //根据ID指定
#define  COMMON_CTRL_ID      0X000   //普通读写指令 ID偏移
#define  COMMON_ANS_ID       0X100    //普通读写反馈ID偏移
#define  SERVER_POS_CTRL_ID  0X200   //位置伺服指令ID偏移
#define  SERVER_SPD_CTRL_ID  0X300   //速度闭环伺服指令ID偏移
#define  SERVER_CUR_CTRL_ID  0X400   //电流伺服指令ID偏移
#define  SERVER_CTRL_ANS_ID  0X500   //伺服控制指令应答ID偏移  依次反馈当前电流、当前速度、当前位置、使能状态、错误代码。

#define  SERVER_STATE_ASK_ID 0X600  //查询关节状态信息，ID偏移
#define  SERVER_STATE_ANS_ID 0X700   //伺服状态查询应答ID偏移  依次反馈错误代码，系统电压，系统温度，使能状态，当前位置。
#define  IAP_CMD_ID          0x7F0   //IAP数据指令

//FDCAN通信 常用数据长度定义
#define  DLC_COMMON_CTRL  2 // 通用控制指令 四个字节组成， CMD index data0 data1

#define  DLC_SERVER_CTRL  4 // 电流、速度、位置伺服控制的数据长度。
#define  DLC_SERVER_CTRL_ANS  16 //对应16个字节 依次反馈当前速度、当前电流、当前电压、使能状态、错误代码。

#define  DLC_SERVER_STATE_ASK  0 // 数据长度为0.
#define  DLC_SERVER_STATE_ANS  12 // 对应12个字节 依次反馈错误代码，系统电压，系统温度，使能状态，当前位置。

//内存控制表宏定义
#define CMDMAP_INDLEN	10 			//内存控制表索引数
#define CMDMAP_SUBLEN	16 			//内存控制表子索引数
#define CMDMAP_LEN		16*10			//内存控制表总长度（字单位）

//主索引地址定义
#define IND_SYS			0x00		//驱动器状态和参数配置
#define IND_CUR         0x01    //当前驱动器电流、速度、位置等控制信息
#define IND_MOT			0x02		//电机模块 及关节全球唯一ID
#define IND_TAG			0x03		//关节目标电流、速度、位置等控制信息
#define IND_LIT			0x04		//关节速度、加速度、位置等限制信息。
#define IND_SEV			0x05		//关节PID配置信息
#define IND_BRK			0x06		//关节抱闸信息
#define IND_ENC 		0x07		//编码器模块相关信息和DRV器件状态信息
#define IND_CAL1        0X08    //参数校准1
#define IND_CAL2        0X09    //参数校准2

//0x0*驱动器参数模块子索引地址定义
#define SYS_ID						0x01		//驱动器ID
#define SYS_MODEL_TYPE 		0x02		//驱动器型号
#define SYS_FW_VERSION		0x03	  //系统版本号
#define SYS_ERROR		 			0x04		//错误代码
#define SYS_VOLTAGE				0x05		//当前输入电压 单位为 0.01V
#define SYS_TEMP	   			0x06  	//当前温度 单位为 0.1°
#define SYS_REDU_RATIO    0x07    //模块减速比
#define SYS_BPS_CANFD_ARB 	 0x08		//CAN总线仲裁域的波特率
#define SYS_BPS_CANFD_DATA 	 0x09	  //CAN总线数据域的波特率
#define SYS_ENABLE_DRIVER    0X0A  //驱动器使能标志   1使能，0不使能
#define SYS_ENABLE_ON_POWER  0X0B // 上电使能驱动器标志  1使能，0不使能
#define SYS_SAVE_TO_FLASH    0X0C //保存数据到flash标志  1保存，0不保存
#define SYS_DEMA_ABSPOS      0X0D // 自动标定绝对位置标志 1标定，0不标定
#define SYS_SET_ZERO_POS     0X0e  //将当前位置设置为零点标志 1设置，0不设置
#define SYS_CLEAR_ERROR      0X0F // 清除错误标志 1清除，0不清除

//0x01
#define CAN_BroadCast_ID  0X00  // 广播ID

//0x02驱动器型号定义
#define J14      0x02    //关节14
#define J17      0x03    //关节17
#define J20      0x04    //关节20
#define J25      0x05    //关节25
#define GRIPPER  0x06    //手爪

//0x04 错误代码
#define  ERR_MASK_FOC            0x0001  //FOC错误
#define  ERR_MASK_OVER_VOLTAGE   0X0002  //系统电压超过安全范围
#define  ERR_MASK_UNDER_VOLTAGE  0X0004  //系统电压低于安全范围
#define  ERR_MASK_OVER_TEMP      0X0008  //温度过高
#define  ERR_MASK_START          0X0010  //启动过程失败
#define  ERR_MASK_ENC            0X0020  //编码器出错
#define  ERR_MASK_OVER_CURRENT   0X0040  //电机电流超过安全范围
#define  ERR_MASK_SOFTWARE       0X0080  //软件错误
#define  ERR_MASK_TEMP           0X0100  //温度传感器出错标志
#define  ERR_MASK_TAG_POS        0X0200  // 目标位置超限
#define  ERR_MASK_DRV8320        0x0400  // DRV8320错误。，需要掉使能清除错误代码
#define  ERR_MASK_TRACK_POS      0x0800  //位置跟踪误差错误
#define  ERR_MASK_CURRENT_DETECT 0x1000  //上电电流传感器错误

//0x08~0x09 CAN总线波特率
#define CAN_BAUD_250K   0 //250KHZ
#define CAN_BAUD_500K   1 //500KHZ
#define CAN_BAUD_1M     2//1MHZ
#define CAN_BAUD_2M     3//2MHZ
#define CAN_BAUD_4M     4//4MHZ
#define CAN_BAUD_5M     5//5MHZ

//0x0a  驱动器使能状态
#define STATE_ENABLE  1  //驱动器使能
#define STATE_DISABLE  0 // 驱动器未使能

//0x0b  驱动器上电使能
#define ON_STATE_ENABLE  1 //驱动器上电使能
#define ON_STATE_DISABLE 0 //驱动器上电不使能

//0x0c  保存数据到flash标志
#define SAVE_FLASH     1  //保存数据到flash
//0x0d  自动标定零位
#define CAL_ZERO_POSE  1  //自动标定0位
//0x0e  将当前位置设置为0位
#define SET_ZERO_POSE  1  //将当前位置设置为0位
//0x0f  清除错误标志
#define CLEAR_ERR      1  //清除错误标志

//0x1*  驱动器电流、速度、位置等控制信息子索引地址定义
#define CUR_CUR_L				0x00		//当前电流低16位 单位mA
#define CUR_CUR_H				0x01		//当前电流高16位
#define CUR_SPEED_L 		0X02    //当前速度低16位，单位units/s
#define CUR_SPEED_H     0X03    //当前速度高16位，
#define CUR_POS_L       0X04    //当前位置高16位，单位units
#define CUR_POS_H       0X05    //当前位置高16位
#define CUR_CAL         0x08    //当前标定状态


//0x2* 电机模块 及关节全球唯一ID子索引地址定义
#define MOT_RES					0x00		//电机内阻 单位毫欧
#define MOT_INDUC				0x01		//电机电感 单位 mH
#define MOT_RATED_VOL		0x02		//电机额定电压  单位0.1V
#define MOT_RATED_CUR		0x03		//电机额定电流  mA
#define MOT_POLES   		0x04		//电机极对数   poles
#define MOT_HALL_VALUE	0x05		//当前霍尔状态


// 关节ID设置
#define MOT_MODEL_ID0   0x0A    //模块全球唯一ID[15:0]
#define MOT_MODEL_ID1   0x0B    //模块全球唯一ID[31:16]
#define MOT_MODEL_ID2   0x0C    //模块全球唯一ID[47:32]
#define MOT_MODEL_ID3   0x0D    //模块全球唯一ID[63:48]
#define MOT_MODEL_ID4   0x0E    //模块全球唯一ID[79:64]
#define MOT_MODEL_ID5   0x0F    //模块全球唯一ID[95:80]


//0x3* 关节目标电流、速度、位置等控制信息子索引地址定义
#define TAG_WORK_MODE   0X00    //工作模式
#define TAG_OPEN_PWM    0X01    //开环模式下的占空比（0~100）
#define TAG_CURRENT_L   0X02    //目标电流低16位 单位mA
#define TAG_CURRENT_H   0X03    //目标电流高16位
#define TAG_SPEED_L     0X04    //目标速度低16位 0.01RPM
#define TAG_SPEED_H     0X05    //目标速度高16位
#define TAG_POSTION_L   0X06    //目标位置低16位 units
#define TAG_POSTION_H   0X07    //目标位置高16位
//#define TAG_POSTION_CAL 0X08    //目标位置补偿值 units
//#define TAG_SPEED_RPM   0X09    //上位机路径规划速度 0.01RPM
#define TAG_SERVO_FLAG  0x08      //目标状态伺服
#define TAG_JOINT_RUN_LEVEL 0x09  // 关节跟随等级

//关节控制模式选择
#define  OPEN_MODE    0X00  //开环控制模式
#define  CUR_MODE     0X01  //电流模式
#define  SPD_MODE     0X02  //速度模式
#define  POS_MODE     0x03  //位置闭环模式

//0x4* 关节速度加速度、位置等限制信息。 子索引地址定义
#define LIT_MAX_CURRENT   0X00    //最大电流 mA
#define LIT_MAX_SPEED     0X01    //最大速度    0.01 RPM
#define LIT_MAX_ACC       0X02    //最大加速度  0.1 RPM/s
#define LIT_MAX_DEC       0X03    //最大减速度  0.1 RPM/S
#define LIT_MIN_POS_L     0X04    //最小位置低16位 units
#define LIT_MIN_POS_H     0X05    //最小位置高16位
#define LIT_MAX_POS_L     0X06    //最大位置低16位 units
#define LIT_MAX_POS_H     0X07    //最大位置高16位
#define IAP_CMD_INDEX       0X09    //IAP更新指令
#define LIT_MAX_POS_DIF_L   0X0B    //最大位置跟踪误差
#define LIT_MAX_POS_DIF_H   0X0C    //最大位置跟踪误差
#define LIT_LOAD_TEST_EN    0X0F   //负载测试使能

// 0x05		//关节PID配置信息。子索引地址定义
#define SEV_PARAME_UPDATE   0X00  //三闭环参数更新标志， 1更新到伺服程序，0，不更新
#define SEV_CURRENT_P       0X01  //电流环P参数
#define SEV_CURRENT_I       0X02  //电流环I参数
#define SEV_CURRENT_D       0X03  //电流环D参数
#define SEV_SPEED_P  				0X04  //速度环P参数
#define SEV_SPEED_I					0X05  //速度环I参数
#define SEV_SPEED_D					0X06  //速度环D参数
#define SEV_SPEED_DS				0X07  //速度环死区
#define SEV_POS_P						0X08  //位置环P参数
#define SEV_POS_I						0X09  //位置环I参数
#define SEV_POS_D  					0X0A  //位置环D参数
#define SEV_POS_DS					0X0B  //位置环死区
#define SEV_POS_SMOOTH       0x0C       //位置平滑系数
#define SEV_SPD_FF           0x0D        //速度前馈系数
#define SEV_CUR_FF           0x0E        //力矩前馈系数

//0X51 //三闭环参数锁定标志


//0X06 //关节抱闸信息  子索引地址定义
#define BRK_RELEASE_CMD     0x01    //刹车释放命令
#define BRK_STATE           0X02    //当前刹车状态

#define  CMD_BRK_USELESS        0X00  //刹车操作无效
#define  CMD_BRK_RELEASE        0X01  //释放刹车
#define  CMD_BRK_LOCK           0X02  //刹车抱死

#define  STATE_BRK_RELEASE      0X00  //释放刹车
#define  STATE_BRK_LOCK         0X01  //抱死刹车

//	0x07		//编码器模块信息  子索引地址定义
#define  ENC_ERR_CUT_L      0X00   //编码器计数器低16位
#define  ENC_ERR_CUT_H      0X01   //编码器计数器高16位
#define  ENC_ICMU_STATE     0X02   //编码器IC_MU状态
#define  ENC_ZERO_POS_OFFSET_L    0X03  //零点位置偏移量低16位  units
#define  ENC_ZERO_POS_OFFSET_H    0X04  //零点位置偏移量高16位  units
#define  ENC_DRV8320_STATE1      0X05  //DRV8320状态寄存器1
#define  ENC_DRV8320_STATE2      0X06  //DRV8320状态寄存器2
#define  ALIEN_STATE           0X07  // FOC伺服错误。默认值为0

// 0x08   IND_CAL1  //电流参数校准信息，子索引地址定义
#define  CAL_CUR_VOL_TEMP_OFFSET				 0X00   // 电流参数校准，校准后存储flash、1：校准，0：不校准
#define  CAL_CUR_OFFSET_1      0X01   // DIER=0 HALLSTATE=1 下的电流偏置值
#define  CAL_CUR_OFFSET_2      0X02   // DIER=0 HALLSTATE=2 下的电流偏置值
#define  CAL_CUR_OFFSET_3      0X03   // DIER=0 HALLSTATE=3 下的电流偏置值
#define  CAL_CUR_OFFSET_4      0X04   // DIER=0 HALLSTATE=4 下的电流偏置值
#define  CAL_CUR_OFFSET_5      0X05   // DIER=0 HALLSTATE=5 下的电流偏置值
#define  CAL_CUR_OFFSET_6      0X06   // DIER=0 HALLSTATE=6 下的电流偏置值
#define  CAL_CUR_K_1           0X07   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_CUR_K_2           0X08   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_CUR_K_3           0X09   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_CUR_K_4           0X0A   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_CUR_K_5           0X0B   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_CUR_K_6           0X0C   // HALLSTATE=1下的电流校准系数*10000
#define  CAL_VOL_OFFSET1       0X0D   // 电压偏置值
#define  CAL_VOL_K             0X0E   // 电压校准系数 k

// 0X09    //校准参数2
#define  CAL_TEMP_OFFSET1      0X00   //温度偏移值1
#define  CAL_TEMP_OFFSET2      0X01   //温度偏移值2
#define  CAL_TEMP_K            0X02   //温度校准系数K

// 状态指示灯的错误定义
#define LED_ERR_NOERR            0X00 //无错误
#define LED_ERR_FLASH_INIT   		 0X01 //初始化flash出错
#define LED_ERR_ADC_SAMPLE     	 0X02 //ADC初始化错误
#define LED_ERR_UNDER_CURRENT     0X03 //过流
#define LED_ERR_TEMP_UP_LOW       0X04 //过温/低温
#define LED_ERR_OVER_VOLTAGE     0X05 //过压
#define LED_ERR_UNDER_VOLTAGE    0X06 //欠压
#define LED_ERR_ENC_INIT         0X07 //编码器初始化错误
#define LED_ERR_HALL_ERR         0X08 //霍尔传感器错误
#define LED_ERR_DRV8320_STATE    0X09 //DRV8320状态错误

#define MY_SGN(value) ((value) > 0 ? 1 : -1)

enum {
    JOINT_DISABLE = 0x00,
    JOINT_ENABLE
};

#endif // CANTYPE_H
