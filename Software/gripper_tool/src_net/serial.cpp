#include "src_net/serial.h"
#include <QDebug>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <QTime>
#include <qapplication.h>

#define INST_REPORT_LEN   (18)

SerialConnection::SerialConnection(QObject *parent) :
    QObject(parent)
{
    _serial = new QSerialPort(this); //构造一个串口设备

    connect(_serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
            SLOT(handleError(QSerialPort::SerialPortError)));

    connect(_serial, SIGNAL(readyRead()), this, SLOT(readData()));
    findSerialDevices();
    m_commonTimer = new QTimer(this);
    connect(m_commonTimer, SIGNAL(timeout()), this, SLOT(slot_commonTimer()));
}

SerialConnection::~SerialConnection()
{
    if(_serial->isOpen())
        _serial->close();

    delete _serial;
}

QStringList SerialConnection::portsList() //串口列表
{
    return _ports;
}

void SerialConnection::findSerialDevices() //查找串口设备
{
    QList<QSerialPortInfo>	info;
    QStringList ports;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
    {
        //Their is some sorting to do for just list the port I want, with vendor Id & product Id
//        qDebug() << port.portName() << port.vendorIdentifier() << port.productIdentifier()
//                 << port.hasProductIdentifier() << port.hasVendorIdentifier() << port.isBusy()
//                 << port.manufacturer() << port.description();
        info.append(port);
        ports.append(port.portName());
    }

    _portInfo.clear();
    _ports.clear();
    for(int i = 0; i < info.size(); i++) {
        _portInfo.append(info.at(i));
        _ports.append(ports.at(i));
    }
}

int SerialConnection::openSerialPort(QSerialPortInfo x)  //打开串口
{
    int error = 0;
    _serial->setPort(x);

    if(!_serial->isOpen())
    {
        if (_serial->open(QIODevice::ReadWrite))
        {
            _serial->setBaudRate(m_baud/*p.baudRate*/);
            _serial->setDataBits(QSerialPort::Data8/*p.dataBits*/);
            _serial->setParity(QSerialPort::NoParity/*p.parity*/);
            _serial->setStopBits(QSerialPort::OneStop/*p.stopBits*/);
            _serial->setFlowControl(QSerialPort::NoFlowControl /*p.flowControl*/);

            emit statusBarMessage(tr("Connected to %1").arg(x.portName())); //状态信号
            emit connectionStateChanged(Connected);
        }
        else
        {
            emit statusBarMessage(tr("Open error"));

            qDebug() << "Serial error: " << _serial->error();

            _serial->close();

            emit serialError();

            error = 1;
        }
    }
    else
    {
        qDebug() << "port already open!";

        error = 0;
    }

    return error;
}

int SerialConnection::openConnection(int index, int baud)
{
    QSerialPortInfo x;
    int foundit = -1;
    int open = false;

    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
    {
        //if( (port.description() == DEVICE_STR3) || (port.description()== DEVICE_STR2))
        {
            qDebug() << port.description();
            foundit++;
            if(foundit==index)
            {
                x = port;
                open = true;
                break;
            }
        }
    }

    qDebug() << "is busy? " << x.isBusy() << "index " << index << " = found " << foundit;

    if(!open)
    {
        qDebug() << "open serial failed ";
        return -1;
    }

    qDebug() << "open serial port " << index << x.portName();
    m_baud = baud;
    //open serial port
    return openSerialPort(x);
}

int SerialConnection::openConnection(QString com, int baud)
{
    QSerialPortInfo x;
    foreach (const QSerialPortInfo &port, QSerialPortInfo::availablePorts())
    {
        qDebug() << port.portName();
        if(port.portName() == com) {
            x = port;
        }
    }
    _serial->setPort(x);

    if(!_serial->isOpen())
    {
        if (_serial->open(QIODevice::ReadWrite))
        {
            // 连接串口
            //_serial->setPortName(com);  // 根据实际情况设置
            _serial->setBaudRate(baud);
            _serial->setDataBits(QSerialPort::Data8);
            _serial->setParity(QSerialPort::NoParity);
            _serial->setStopBits(QSerialPort::OneStop);
            _serial->setFlowControl(QSerialPort::NoFlowControl);
            qDebug() << "Port" << _serial->portName() << "opened successfully.";
            emit statusBarMessage(tr("连接成功"));
            emit connectionStateChanged(Connected); //连接成功
        } else {
            qDebug() << "Failed to open port" << _serial->portName() << ", error:" << _serial->errorString();
            emit statusBarMessage(QString("打开错误:%1").arg(_serial->errorString()));
            emit serialError();
            //emit error();
            _serial->close();
        }
    }
    return 0;
}

void SerialConnection::closeConnection()
{
    if(_serial->isOpen()) {
        _serial->close();
        emit statusBarMessage(tr("断开连接"));
        emit connectionStateChanged(Disconnected);
        qDebug() << "Port" << _serial->portName() << "close successfully.";
    }
}

void SerialConnection::writeData(const QByteArray &data)
{
    if(_serial->isOpen())
    {
        _serial->write(data);
    }
}

void SerialConnection::handleError(QSerialPort::SerialPortError error)
{
    static int i = 0;
    i++;
    if (error == QSerialPort::ResourceError) {

        _serial->close();
        _processingData = true;
    }
    if(i == 3)
    {
      emit connectionStateChanged(ConnectionFailed);
     i = 0;
    }

}

quint8 SerialConnection::sum_check(QByteArray data, quint16 len)
{
    quint32 sum = 0;
    for(int i = 0; i < len; i++)
    {
        sum += (quint8)data.at(i);
    }

    return sum&0x000000FF;
}

bool SerialConnection::isFrame485Complete(const QByteArray &data)
{
    if (data.size() < 7) {
        return false;
    }
    // 数据长度 + 固定头2字节 +（ID，长度） + 和校验
    int frameLength = (quint8)data.at(3) + 5;
    return data.size() >= frameLength;
}
bool SerialConnection::isFrameRTUComplete(const QByteArray &data)
{
    if (data.size() < 5) {
        return false;
    }

    // 获取功能码
    quint8 functionCode = static_cast<quint8>(data.at(1));

    // 处理错误响应
    if (functionCode & 0x80) {
        return data.size() >= 5; // 错误响应固定长度为5
    }

    // 处理正常响应
    quint8 byteCount = static_cast<quint8>(data.at(2));
    int expectedLength = 3 + byteCount + 2; // 从站地址 + 功能码 + 数据字节数 + 数据 + CRC

    return data.size() >= expectedLength;
}
uint16_t SerialConnection::calculateCRC(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < data.size(); ++i) {
        crc ^= static_cast<uint8_t>(data.at(i));
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
void SerialConnection::readData(void)
{
    buffer.append(_serial->readAll());

    processBuffer();
    m_commonTimer->setSingleShot(true);
    m_commonTimer->start(50);
}

void SerialConnection::processBuffer()
{
    while (!buffer.isEmpty()) {
        qDebug() << buffer.toHex() << buffer.size();
        if(buffer.size() < 4)
            break;

        if((quint8)buffer.at(0) == 0xEE && (quint8)buffer.at(1) == 0x16) {
            // 检查是否有完整的帧
            if (isFrame485Complete(buffer)) {
                // 提取完整的帧
                int frameLength = (quint8)buffer.at(3) + 5; // 数据长度 + 固定头3字节（地址，功能码，长度） + CRC2字节
                //qDebug() << "1" << frameLength;
                if (buffer.size() < frameLength) {
                    break; // 数据帧尚未完整接收
                }
                QByteArray frame;
                //qDebug() << "2" << frameLength;
                frame.resize(frameLength);
                //qDebug() << "3" << frameLength;
                frame = buffer.left(frameLength);
                buffer.remove(0, frameLength);
                //qDebug() << "4" << frame.size();
                quint8 sum_value = sum_check(frame.mid(2, (quint8)frame.at(3)+2), (quint8)frame.at(3)+2);
                if(sum_value == (quint8)frame.at(frameLength-1))
                {
                    emit sendnewdata(frame);
                } else {
                    qWarning() << "SUM check error!" << sum_value << (quint8)frame.at(frameLength-1);
                }
                //qDebug() << "5" << frame.size();
                m_commonTimer->stop();
            } else {
                break; // 如果没有完整的帧，退出处理循环，等待更多数据
            }
        } else {
            // 检查是否有完整的帧
            if (isFrameRTUComplete(buffer)) {
                QByteArray frame = buffer.left(buffer.size());
                buffer.remove(0, buffer.size());
                //qDebug() << frame.toHex();
                // 校验 CRC
                uint16_t receivedCRC = qFromLittleEndian<quint16>(reinterpret_cast<const uchar*>(frame.constData() + frame.size() - 2));
                uint16_t calculatedCRC = calculateCRC(frame.left(frame.size() - 2));

                if (receivedCRC == calculatedCRC) {
                    // 处理完整且校验通过的帧
                    emit sendnewdata(frame);
                } else {
                    qWarning() << "CRC error!";
                }
                m_commonTimer->stop();
            } else {
                // 如果没有完整的帧，退出处理循环，等待更多数据
                break;
            }
        }

    }
}

void SerialConnection::SendCmdData(QByteArray srcBuffer)
{
    writeData(srcBuffer);
}

void SerialConnection::slot_commonTimer()
{
    buffer.clear();
}
