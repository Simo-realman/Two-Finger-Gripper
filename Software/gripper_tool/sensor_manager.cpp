#include "sensor_manager.h"
#include <QMutex>
#include <QDebug>
#include <QMessageBox>

static QMutex ConfMutex;
static sensor_manager * m_instance;

sensor_manager::sensor_manager(QObject *parent) :
    QObject(parent)
{

}
sensor_manager::~sensor_manager()
{

}

sensor_manager * sensor_manager::getInstance()
{
    if(NULL == m_instance)
    {
        ConfMutex.lock();
        if(NULL == m_instance)
        {
            m_instance = new sensor_manager();
        }
        ConfMutex.unlock();
    }
    return m_instance;
}

/* 保存设备信息 */
void sensor_manager::save_dev_info(QByteArray _bSensorData)
{
    int index = -1;
    int ListSize = m_SensorInforlist.size();
    quint8 u8SensorType = (quint8)_bSensorData[7];
    quint8 u8SensorID = (quint8)_bSensorData[8];
    cSensorInfor SensorInst;

    for(int i=0; i<ListSize; i++)
    {
        if(m_SensorInforlist[i].sensorID == u8SensorID)
        {
            index = i;
            break;
        }
    }
    if(index == -1) //未添加此设备，加入列表
    {
        SensorInst.sensortype     = (quint8)_bSensorData[7]; //传感器类型
        SensorInst.sensorID       = (quint8)_bSensorData[8];
        SensorInst.sensorindex    = 0;
        SensorInst.netid          = 0;
        SensorInst.nodeaddress[0] = (quint8)_bSensorData[2];
        SensorInst.nodeaddress[1] = (quint8)_bSensorData[3];
        SensorInst.nodeaddress[2] = (quint8)_bSensorData[4];
        SensorInst.nodeaddress[3] = (quint8)_bSensorData[5];

        m_SensorInforlist.append(SensorInst);
    }
}
/* 下发控制指令 */
void sensor_manager::send_485_data(quint8 _id, quint8 _cmd, QByteArray &data, quint16 len)
{
    int i = 0;
    QByteArray CmdData;
    CmdData.resize(len+6);

    CmdData[0] = 0xEB;
    CmdData[1] = 0x90;
    CmdData[2] = _id;
    CmdData[3] = len+1;
    CmdData[4] = _cmd;
    for(i = 0; i < len; i++)
    {
       CmdData[i+5] = data[i];
    }
    CmdData[len+5] = sum_check(CmdData.mid(2,len+3), len+3);

    qDebug()<< CmdData.toHex() << CmdData.length();
    emit sensorCtrol(CmdData);
    emit sendBack(CmdData);

}
/* 下发控制指令 */
void sensor_manager::send_485_data(quint8 _id, quint8 _cmd, quint8 *data, quint16 len)
{
    int i = 0;
    QByteArray CmdData;
    CmdData.resize(len+6);

    CmdData[0] = 0xEB;
    CmdData[1] = 0x90;
    CmdData[2] = _id;
    CmdData[3] = len+1;
    CmdData[4] = _cmd;
    for(i = 0; i < len; i++)
    {
       CmdData[i+5] = data[i];
    }
    CmdData[len+5] = sum_check(CmdData.mid(2,len+3), len+3);

    qDebug()<< CmdData.toHex() << CmdData.length();
    emit sensorCtrol(CmdData);
    emit sendBack(CmdData);

}
/* 下发控制指令 */
void sensor_manager::send_rtu_data(QByteArray &data)
{
    qDebug()<< data.toHex() << data.length();
    emit sensorCtrol(data);
    emit sendBack(data);
}

quint8 sensor_manager::sum_check(QByteArray data, quint16 len)
{
    quint32 sum = 0;
    for(int i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum&0x000000FF;
}
