#include "modbusinterface.h"

ModbusInterface::ModbusInterface(QObject *parent) : QObject(parent)
{

}

ModbusInterface::~ModbusInterface()
{

}

uint16_t ModbusInterface::calculateCRC(const QByteArray &data)
{
    uint16_t crc = 0xFFFF;
    for (const char byte : data) {
        crc ^= static_cast<uint8_t>(byte);
        for (int j = 0; j < 8; j++) {
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

QByteArray ModbusInterface::createModbusRequest(quint8 slaveAddress, quint8 functionCode, const QByteArray &data)
{
    QByteArray request;
    request.append(slaveAddress);
    request.append(functionCode);
    request.append(data);

    uint16_t crc = calculateCRC(request);
    request.append(crc & 0xFF);
    request.append((crc >> 8) & 0xFF);

    return request;
}

QByteArray ModbusInterface::readCoils(quint8 slaveAddress, quint16 startAddress, quint16 quantity)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(quantity >> 8);
    data.append(quantity & 0xFF);

    QByteArray request = createModbusRequest(slaveAddress, 0x01, data);
    return request;
}

QByteArray ModbusInterface::readDiscreteInputs(quint8 slaveAddress, quint16 startAddress, quint16 quantity)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(quantity >> 8);
    data.append(quantity & 0xFF);

    QByteArray request = createModbusRequest(slaveAddress, 0x02, data);
    return request;
}

QByteArray ModbusInterface::readHoldingRegisters(quint8 slaveAddress, quint16 startAddress, quint16 quantity)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(quantity >> 8);
    data.append(quantity & 0xFF);

    QByteArray request = createModbusRequest(slaveAddress, 0x03, data);
    return request;
}

QByteArray ModbusInterface::readInputRegisters(quint8 slaveAddress, quint16 startAddress, quint16 quantity)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(quantity >> 8);
    data.append(quantity & 0xFF);

    QByteArray request = createModbusRequest(slaveAddress, 0x04, data);
    return request;
}

QByteArray ModbusInterface::writeSingleCoil(quint8 slaveAddress, quint16 address, bool value)
{
    QByteArray data;
    data.append(address >> 8);
    data.append(address & 0xFF);
    data.append(value ? 0xFF : 0x00);
    data.append((quint8)0x00);

    QByteArray request = createModbusRequest(slaveAddress, 0x05, data);
    return request;
}

QByteArray ModbusInterface::writeSingleRegister(quint8 slaveAddress, quint16 address, quint16 value)
{
    QByteArray data;
    data.append(address >> 8);
    data.append(address & 0xFF);
    data.append(value >> 8);
    data.append(value & 0xFF);

    QByteArray request = createModbusRequest(slaveAddress, 0x06, data);
    return request;
}

QByteArray ModbusInterface::writeMultipleCoils(quint8 slaveAddress, quint16 startAddress, const QVector<bool> &values)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(values.size() >> 8);
    data.append(values.size() & 0xFF);

    // 计算字节数并填充输出值
    int byteCount = (values.size() + 7) / 8;
    data.append(byteCount);

    QByteArray coilData(byteCount, 0);
    for (int i = 0; i < values.size(); ++i) {
        if (values[i]) {
            coilData[i / 8] = coilData[i / 8]|(1 << (i % 8));
        }
    }
    data.append(coilData);

    QByteArray request = createModbusRequest(slaveAddress, 0x0F, data);
    return request;
}

QByteArray ModbusInterface::writeMultipleRegisters(quint8 slaveAddress, quint16 startAddress, const QVector<quint16> &values)
{
    QByteArray data;
    data.append(startAddress >> 8);
    data.append(startAddress & 0xFF);
    data.append(values.size() >> 8);
    data.append(values.size() & 0xFF);

    // 计算字节数并填充寄存器值
    int byteCount = values.size() * 2;
    data.append(byteCount);

    for (int i = 0; i < values.size(); ++i) {
        data.append(values[i] >> 8);
        data.append(values[i] & 0xFF);
    }

    QByteArray request = createModbusRequest(slaveAddress, 0x10, data);
    return request;
}
