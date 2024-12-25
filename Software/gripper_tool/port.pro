#-------------------------------------------------
#
# Project created by QtCreator 2021-05-17T20:18:37
#
#-------------------------------------------------

QT       += core gui widgets serialport sql charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
greaterThan(QT_MAJOR_VERSION, 5): QT += core5compat printsupport

TARGET = gripper_tools
TEMPLATE = app

DESTDIR = $$PWD/bin
# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += $$PWD/third
include ($$PWD/third/third.pri)

SOURCES += \
    devicemanager.cpp \
    main.cpp \
    mainwindow.cpp \
    modbusinterface.cpp \
    tipwidget.cpp \
    src_net/serial.cpp \
    sensor_manager.cpp\
    dataqueue.cpp

HEADERS += \
    devicemanager.h \
    mainwindow.h \
    modbusinterface.h \
    dataqueue.h \
    tipwidget.h \
    src_net/sensordata.h \
    src_net/serial.h \
    sensor_manager.h


FORMS += \
        mainwindow.ui

RESOURCES += \
    image.qrc

#把所有警告都关掉
CONFIG += warn_off
