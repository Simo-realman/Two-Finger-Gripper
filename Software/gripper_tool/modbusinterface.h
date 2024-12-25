#ifndef MODBUSINTERFACE_H
#define MODBUSINTERFACE_H

#include <QObject>
#include <QByteArray>
#include <QDebug>

class ModbusInterface : public QObject
{
    Q_OBJECT

public:
    explicit ModbusInterface(QObject *parent = nullptr);
    ~ModbusInterface();
    uint16_t calculateCRC(const QByteArray &data);
    QByteArray createModbusRequest(quint8 slaveAddress, quint8 functionCode, const QByteArray &data);

    QByteArray readCoils(quint8 slaveAddress, quint16 startAddress, quint16 quantity);
    QByteArray readDiscreteInputs(quint8 slaveAddress, quint16 startAddress, quint16 quantity);
    QByteArray readHoldingRegisters(quint8 slaveAddress, quint16 startAddress, quint16 quantity);
    QByteArray readInputRegisters(quint8 slaveAddress, quint16 startAddress, quint16 quantity);
    QByteArray writeSingleCoil(quint8 slaveAddress, quint16 address, bool value);
    QByteArray writeSingleRegister(quint8 slaveAddress, quint16 address, quint16 value);
    QByteArray writeMultipleCoils(quint8 slaveAddress, quint16 startAddress, const QVector<bool> &values);
    QByteArray writeMultipleRegisters(quint8 slaveAddress, quint16 startAddress, const QVector<quint16> &values);
signals:

};

#endif // MODBUSINTERFACE_H
