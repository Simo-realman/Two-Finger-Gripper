#ifndef SENSORDATA_H
#define SENSORDATA_H
#include <QString>
#include <QtGlobal>

#define CMDID_UP								0x6A
#define CMDID_DOWN								0x80
#define CMDID_MODE								0x2A

enum eSensorID{
    SENSOR_S_DETECT         ,
};

enum eSensorType
{
    TYPE_A_DETECT = 0x01,
    TYPE_D_DETECT       ,
    TYPE_EXECUTE        ,
    TYPE_SPEL   = 0x10  ,
};
#define COMDATAMAXLENGTH 18

class cSensorInfor{

public:
    quint8 netid;
    quint8 nodeaddress[4];
    quint8 sensorindex;
    quint8 sensortype;
    quint8 sensorID;
    cSensorInfor():netid(0),sensorindex(0),sensortype(0),sensorID(0){}
};

#endif // SENSORDATA_H
