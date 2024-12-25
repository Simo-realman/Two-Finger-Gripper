#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include <QObject>
#include <QThread>
#include <QList>
#include "dataqueue.h"

class DeviceManager : public QThread
{
    Q_OBJECT
public:
    DeviceManager();
    ~DeviceManager();

    void sendData(quint8 _id, quint8 _cmd, QByteArray &data, quint16 len);

private:
    void sendQueueTask(void);

    void send_485_data(quint8 _id, quint8 _cmd, quint8 *data, quint16 len);
    quint8 sum_check(QByteArray data, quint16 len);
signals:
    void sensorCtrol(QByteArray _bCmdData);
    void sendBack(QByteArray _bCmdData);
protected:
    void run() override;
};

#endif // DEVICEMANAGER_H
