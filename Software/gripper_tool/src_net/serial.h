#ifndef SERIALCONNECTION_H
#define SERIALCONNECTION_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QStringList>
#include <QTimer>
#include <QtEndian>

#define DEVICE_STR ("STMicroelectronics Virtual COM Port")
#define DEVICE_STR1 ("USB-SERIAL CH340")
#define DEVICE_STR2 ("Silicon Labs CP210x USB to UART Bridge")
#define DEVICE_STR3 ("Prolific USB-to-Serial Comm Port")
/**
* @brief SerialConnection
*        Constructor, it initialises the Serial Connection its parts
*        it is used for managing the COM port connection.
*/
class SerialConnection : public QObject
{
    Q_OBJECT
public:
    explicit SerialConnection(QObject *parent = 0);
    ~SerialConnection();

    enum ConnectionState
    {
        Disconnected = 0,
        Connecting,
        Connected,
        ConnectionFailed
    };

    typedef enum
    {
        DATA_HEAD_H = 0,
        DATA_HEAD_L,
        DATA_ID,
        DATA_LEN,
        DATA_BODY,
        DATA_END,
    }eRxStep;

    void TransmitData(void);

    bool CRC_Calculate(QByteArray pBuf);



    int openSerialPort(QSerialPortInfo x); //open selected serial port

    QStringList portsList(); //return list of available serial ports (list of ports with tag/anchor connected)

    QSerialPort* serialPort() { return _serial; }

    quint16 getCrcValue(QByteArray pBuf, quint8 len);
    int _TransferredMeaning(QByteArray pSrc, quint8 Srclength);
signals:
    void clearTags();
    void serialError(void);
    void getCfg(void);
    void nextCmd(void);

    void statusBarMessage(QString status);
    void connectionStateChanged(int state);
    void serialOpened(QString, QString);

    void sendnewdata(QByteArray data); //发送读取到的数据

public slots:
    void closeConnection();
    int  openConnection(int index, int baud);
    int openConnection(QString com, int baud);
    void readData(void);
    void SendCmdData(QByteArray srcBuffer);
    void findSerialDevices(); //find any tags or anchors that are connected to the PC
protected slots:
    void writeData(const QByteArray &data);
    void handleError(QSerialPort::SerialPortError error);
private slots:
    void slot_commonTimer();

private:
    QSerialPort *_serial;
    QList<QSerialPortInfo>	_portInfo ;
    QStringList _ports;
    QList<QByteArray> _cmdList ;
    QString _connectionVersion;
    QString _conncectionConfig;
    bool _processingData;
    int m_baud = 115200;
    eRxStep    rxStep;
    QByteArray recvBuffer;
    QByteArray buffer;
    QTimer *m_commonTimer = nullptr;

    quint8 sum_check(QByteArray data, quint16 len);
    void processBuffer();
    uint16_t calculateCRC(const QByteArray &data);
    bool isFrameRTUComplete(const QByteArray &data);
    bool isFrame485Complete(const QByteArray &data);
};

#endif // SERIALCONNECTION_H
