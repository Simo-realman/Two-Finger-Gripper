#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qthread.h>
#include <QTime>
#include <QFile>
#include "modbusinterface.h"
#include "src_net/serial.h"
#include "src_net/sensordata.h"
#include "qcustomplot.h"
#include "devicemanager.h"

typedef unsigned char   uint8;
typedef char			int8;
typedef unsigned short  uint16;
typedef short			int16;
typedef unsigned int    uint32;
typedef int				int32;

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


#define RM_CURRENT_MAX      24000  //电流做大值 = 额定电流(800mA)x3 mA
#define RM_FORCE_MIN     		(50) //力最小值
#define RM_FORCE_MAX     		(1000) //力做大值
#define RM_SPEED_MIN     		(1) //速度最小值 1%
#define RM_SPEED_MAX     		(1000) //速度最大值 100%
#define RM_POS_MIN     	 		(0) //位置最小值 0mm
#define RM_POS_MAX       		(1000) //位置最大值 65mm
#define RM_RANGE_MIN     	 	(0) //位置最小值 0mm
#define RM_RANGE_MAX       	(65) //位置最大值 65mm
#define RM_ANGLE_MIN     	 	(0) //角度最小值
#define RM_ANGLE_MAX       	(87) //角度最大值

enum {
    E_STATE_STOP_MAX   			= 0x01, //最大且空闲
    E_STATE_STOP_MIN		    = 0x02, //最小且空闲
    E_STATE_STOP_NONE   		= 0x03, //停止且空闲
    E_STATE_PLAN_CLOSE   		= 0x04, //正在闭合
    E_STATE_PLAN_OPEN 			= 0x05, //正在张开
    E_STATE_PLAN_PAUSE   		= 0x05, //力控过程中遇遇到力暂停
    E_STATE_PLAN_STOP 			= 0x06, //规划急停
};

//状态返回
typedef enum {
    CMD_MC_OK          			= 0x01, //正常
    CMD_MC_OUT_RANGE  						, //超限位
    CMD_MC_RUNING  								, //运行中
    CMD_MC_ERR	       			= 0x55,
}e_cmd_state_type;
//夹爪协议
typedef enum {
    CMD_MC_PARA_SAVE 						= 0x01, //参数保存到内部闪存，掉电不丢失
    CMD_MC_PARA_DEFAULT					= 0x02, //恢复默认参数
    CMD_MC_PARA_BAUD_SET				= 0x03, //设置波特率
    CMD_MC_PARA_ID_SET 					= 0x04, //设置夹爪ID
    CMD_MC_PARA_ID_GET 					= 0x05, //读取夹爪ID
    CMD_MC_MOTOR_ID_SET 					= 0x06, //设置关节ID

    CMD_MC_READ_FORCE_ACK				= 0x0A, //读取夹爪力数据

    CMD_MC_MOVE_CATCH_XG				= 0x10, //以设置的速度和力控阈值去夹取
    CMD_MC_MOVE_RELEASE 				= 0x11, //以设置的速度松开
    CMD_MC_SET_EG_PARA 					= 0x12, //设置夹爪开口的最大最小值
    CMD_MC_READ_EG_PARA 				= 0x13, //读取夹爪开口的最大最小值
    CMD_MC_READ_EG_STATE 				= 0x14, //--(预留)
    CMD_MC_MOVE_STOPHERE 				= 0x16, //急停
    CMD_MC_ERROR_CLR            = 0x17, //清除错误
    CMD_MC_MOVE_CATCH2_XG 			= 0x18, //以设置的速度和力控阈值持续夹取
    CMD_MC_SET_FORCE_PARA 			= 0x19, //设置力限位
    CMD_MC_GET_FORCE_PARA 			= 0x1A, //读取力限位
    CMD_MC_SET_DIO_MODE 				= 0x1B, //设置IO输出模式
    CMD_MC_SET_DIO_OUT 					= 0x1C, //设置IO输出电平
    CMD_MC_GET_DIO_STATE				= 0x1D, //读取IO输出电平

    CMD_MC_READ_EG_RUNSTATE 		= 0x41, //读取夹爪运行状态
    CMD_MC_READ_SYSTEM_PARAM 		= 0x42, //读取夹爪系统参数
    CMD_MC_READ_MOTOR_STATE 		= 0x43, //读取夹爪电机状态

    CMD_MC_SEEKPOS 							= 0x54, //设置夹爪开口度

    CMD_MC_READ_ACTPOS 					= 0xD9, //读取夹爪开口度
    CMD_MC_SET_KEY_ENABLE				= 0xDA, //设置IO控制使能
    CMD_MC_SET_ZERO   					= 0xDB, //设置零位
    CMD_MC_SERVO_MOVE   				= 0xDC, //运动校准

    CMD_MC_SEEKPOS_SPEED_FORCE 	= 0xE5, //设置夹爪开口度、速度、力控阈值

    CMD_MC_UPGRADE              = 0xFE, //升级
}e_485_cmd_type;

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

typedef enum {
    E_UPGRADE_STOP 					= 0x00, /*不升级 APP-BOOT*/
    E_UPGRADE_SART 					= 0x01, /*升级 APP-BOOT*/
    E_UPGRADE_IMG_REQ 			= 0x02, /*请求升级包 BOOT-APP*/
    E_UPGRADE_FINISH				= 0x03, /*升级完成 BOOT-APP*/
    E_UPGRADE_FAILED 				= 0x04, /*升级失败 BOOT-APP*/
    E_UPGRADE_RESTART 			= 0x05, /*升级请求 BOOT-APP*/
    E_UPGRADE_AGREE_RESTART = 0x06, /*同意升级请求 APP-BOOT*/
}e_upgrade_index;

//线圈地址
typedef enum /**/
{
    MB_COILS_DIO_MODE_0 	 = 0x0001, /*dio0模式 (rw)*/
    MB_COILS_DIO_STATE_0 				   , /*dio0状态 (rw)*/
    MB_COILS_DIO_MODE_1 	         , /*dio1模式 (rw)*/
    MB_COILS_DIO_STATE_1 				   , /*dio1状态 (rw)*/
} e_coils_info_map;

//设备信息地址表-保持寄存器
typedef enum /**/
{
    MB_REG_PARAM_SAVE 	 = 0x0001, /*保存参数 (rw)*/
    MB_REG_PARAM_DEFAULT 				 , /*参数恢复默认 (rw)*/
    MB_REG_DEV_ID 							 , /*设置设备ID (rw)*/
    MB_REG_UART_BAUD 						 , /*设置串口波特率 (rw)*/

    MB_REG_CATCH 								 , /*力控夹取 (rw)*/
    MB_REG_LOOSE 								 , /*松开 (rw)*/
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

typedef struct{
    int m_iapEnable = 0;
    int m_iapCnt = 0;
    int m_iapSquence = 0;
    int m_iapTimeout = 0;
    QFile m_iapFile;
}s_iap_state;

//该结构体用于存储哪条曲线需要绘制
typedef struct
{
    //实时界面
    bool currentTarget;     //目标电流
    bool currentReal;       //实际电流
    bool speedTarget;       //目标速度
    bool speedReal;         //实际速度
    bool posTarget;         //目标位置
    bool posReal;           //实际位置
}WAVE_ENABLE;

namespace Ui {
class MainWindow;
}


class CustomComboBox : public QComboBox {
    Q_OBJECT

public:
    CustomComboBox(QWidget *parent = nullptr) : QComboBox(parent) {}

signals:
    void popupShown();

protected:
    void showPopup() override {
        emit popupShown();
        QComboBox::showPopup();
    }
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    SerialConnection serial;
    int updateDeviceList();
    void cmd_485_data_process(quint8 _cmd, QByteArray _data);
signals:
    void sig_openSerial(QString com, int baud);
    void StartUartDev_Signal(int index, int baud);
    void CloseUartDev_Signal();
    void sig_findDev();
private slots:
    void on_connect_btn_clicked();
    void RecvSensorData(QByteArray _bSensorData);
    void DispConnectState_Slot(int state);
    void DispDate_Slot(void);
    void slot_btnParamClick();
    void slot_sendBack(QByteArray _bSensorData);
    void on_cbxProtocol_currentIndexChanged(int index);

    void on_spbxComID_valueChanged(int arg1);

    void slot_btnIAPClick();
    void slot_timIAP();
    void on_checkBox_stateChanged(int arg1);
    void slot_timSendData();
    void on_horizontalSlider_valueChanged(int value);

    void slot_rdbWave(int state);
    void slot_btnWave();
    void slot_plotTimer();
    void slot_statusBarMessage(QString msg);

    void on_cbxPlan_stateChanged(int arg1);

    void on_cbxIOEnable_stateChanged(int arg1);

    void onShowPopup();
private:
    Ui::MainWindow *ui;
    QThread *_serialThread;
    QTimer *m_tmDate;
    QTimer *m_sendData;

    DeviceManager * m_deviceManager = nullptr;

    int m_connectState = 0;

    CustomComboBox *comBox;

    QTimer *m_timIAP;
    QFile m_iapFile;

    int m_iapEnable = 0;
    int m_iapOffset = 0;
    int m_iapFileSize = 0;
    int m_iapSquence = 0;
    int m_iapTimeout = 0;
    int m_iapType = 0;
    bool m_iapStartFlag = false;
    int m_iapStartCount = 0;

    ModbusInterface m_mbCreate;
    int online_timeout = 20;
    quint8 m_devId = 0x01;
    int m_protocol = 0;

    float currentReal = 0, speedReal = 0, angleReal = 0, postionReal = 0, tempReal = 0, voltageReal = 0;
    // 错误标志
    QMap<quint16, QString> m_errorMap = {{ (0x0001<<0), "FOC错误" }, \
                                         { (0x0001<<1), "过压" }, \
                                         { (0x0001<<2), "欠压" }, \
                                         { (0x0001<<3), "过温" }, \
                                         { (0x0001<<4), "启动错误" }, \
                                         { (0x0001<<5), "编码器错误" }, \
                                         { (0x0001<<6), "过流" }, \
                                         { (0x0001<<7), "软件错误" }, \
                                         { (0x0001<<8), "温度传感器错误" }, \
                                         { (0x0001<<9), "目标位置超限" }, \
                                         { (0x0001<<10), "DRV8320错误" }, \
                                         { (0x0001<<11), "位置跟踪误差" }, \
                                         { (0x0001<<12), "电流检测错误" }, \
                                         { (0x0001<<13), "自检错误" }, \
                                         { (0x0001<<14), "位置指令超限" }, \
                                         { (0x0001<<15), "多圈丢数" } };
    // 曲线显示定时器
    QTimer *m_plotTimer;
    // 曲线
    QCPGraph *m_graphCurrentReal;
    QCPGraph *m_graphCurrentTarget;
    QCPGraph *m_graphSpeedReal;
    QCPGraph *m_graphSpeedTarget;
    QCPGraph *m_graphPosReal;
    QCPGraph *m_graphPosTarget;
    WAVE_ENABLE m_waveEnable = {false,false,false,false,false,false};
    // 滚动显示标志位
    bool m_waveRoll = true;
    // 曲线x轴计数
    double m_lineCnt = 0;

    QList<QString> m_listBaud = {"9600", "19200", "38400", "57600", "115200", "460800"};

    QList<QString> m_listState = {"最大且空闲", "最小且空闲", "停止且空闲", "正在闭合", "正在张开", "力控过程中遇到力暂停", "规划急停"};

    void initSlot();
    quint8 sum_check(quint8 *data, quint16 len);
    QString byteArrayToHexString(const QByteArray &data);
    void msleep(int msec);
    void cmd_rtu_data_process(quint8 _func, QByteArray _data);
    float byteToFloat(QByteArray data);
    void iapStateInit();
    QString convertVersion(int version);
    void initCustomPlot();
};

#endif // MAINWINDOW_H
