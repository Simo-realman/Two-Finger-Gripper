#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <QObject>

#include "src_net/sensordata.h"

class sensor_manager: public QObject
{
    Q_OBJECT
public:
    //sensor_manager();
    explicit sensor_manager(QObject *parent = 0);
    ~sensor_manager();
public:
    static sensor_manager * getInstance();
    void save_dev_info(QByteArray _bSensorData);
    void send_485_data(quint8 _id, quint8 _cmd, QByteArray &data, quint16 len);
    void send_rtu_data(QByteArray &data);
    void send_485_data(quint8 _id, quint8 _cmd, quint8 *data, quint16 len);
signals:
    void sensorCtrol(QByteArray _bCmdData);
    void sendBack(QByteArray _bCmdData);
private:
    QList <cSensorInfor> m_SensorInforlist; //传感器数据链表
    quint8 sum_check(QByteArray data, quint16 len);
};


#define SERIAL sensor_manager::getInstance()
#endif // SENSOR_MANAGER_H
