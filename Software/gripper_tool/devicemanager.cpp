#include "devicemanager.h"
#include "sensor_manager.h"
#include <QDebug>

DeviceManager::DeviceManager()
{
}

DeviceManager::~DeviceManager()
{
    this->quit();
    this->wait();
}

void DeviceManager::sendData(quint8 _id, quint8 _cmd, QByteArray &data, quint16 len)
{
    queue_data frame;
    frame.id = _id;
    frame.cmd = _cmd;
    frame.len = len;
    for(int i = 0; i < len; i++) {
        frame.data[i] = (quint8)data.at(i);
    }
    if(!DATAQUEUE->FullQueue())
    {
        DATAQUEUE->Enqueue(&frame);
    }
}
quint8 DeviceManager::sum_check(QByteArray data, quint16 len)
{
    quint32 sum = 0;
    for(int i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum&0x000000FF;
}
/* 下发控制指令 */
void DeviceManager::send_485_data(quint8 _id, quint8 _cmd, quint8 *data, quint16 len)
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

    //qDebug()<< CmdData.toHex() << CmdData.length();
    emit sensorCtrol(CmdData);
    emit sendBack(CmdData);

}

void DeviceManager::sendQueueTask()
{
    queue_data frame;
    if(!DATAQUEUE->EmptyQueue())
    {
        DATAQUEUE->Dequeue(&frame);
        this->send_485_data(frame.id, frame.cmd, frame.data, frame.len);
        //qDebug() << "send data" << frame.id << frame.cmd;
    }
}

void DeviceManager::run()
{
    while(true) {
        sendQueueTask();
        QThread::msleep(5);
    }
}
