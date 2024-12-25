#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "sensor_manager.h"
#include "tipwidget.h"
#include "dataqueue.h"
#include <QFileDialog>
#include <QDebug>
#include <QDateTime>
#include <QMessageBox>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("睿尔曼两指夹爪上位机 v1.0.3");

    /* 串口相关 */

    _serialThread = new QThread(this);
    //connect(this,SIGNAL(StartUartDev_Signal(int, int)),&serial,SLOT(openConnection(int, int)));
    connect(this,SIGNAL(sig_openSerial(QString, int)),&serial,SLOT(openConnection(QString, int)));

    connect(this,SIGNAL(CloseUartDev_Signal()),&serial,SLOT(closeConnection()));
    connect(_serialThread, SIGNAL(started()),&serial, SLOT(readData()));
    connect(_serialThread, SIGNAL(finished()), &serial, SLOT(deleteLater()));
    connect(&serial,SIGNAL(sendnewdata(QByteArray)),this,SLOT(RecvSensorData(QByteArray)));
    connect(&serial,SIGNAL(statusBarMessage(QString)),this,SLOT(slot_statusBarMessage(QString)));
    connect(sensor_manager::getInstance(),SIGNAL(sensorCtrol(QByteArray)),&serial,SLOT(SendCmdData(QByteArray)));
    connect(&serial,SIGNAL(connectionStateChanged(int)),this,SLOT(DispConnectState_Slot(int)));
    serial.moveToThread(_serialThread);
    _serialThread->start();
    connect(sensor_manager::getInstance(),SIGNAL(sendBack(QByteArray)),this,SLOT(slot_sendBack(QByteArray)));
    connect(this,SIGNAL(sig_findDev()),&serial,SLOT(findSerialDevices()));

    comBox = new CustomComboBox(this);
    ui->horizontalLayout_13->insertWidget(1, comBox);
    ui->horizontalLayout_13->setStretch(1, 5);
    connect(comBox, &CustomComboBox::popupShown, this, &MainWindow::onShowPopup);
    updateDeviceList();

    DATAQUEUE->CreateQueue(QUEUE_SIZE);

    /* 背景图片 */
    //QPalette palette;
    //palette.setBrush(backgroundRole(),QBrush(QPixmap(":/image/mainpage.jpg")));
    //this->setPalette(palette);

    /* 状态更新定时器 */
    m_tmDate = new QTimer(this);
    connect(m_tmDate,SIGNAL(timeout()),this, SLOT(DispDate_Slot()));
    m_tmDate->start(1000); //1s

    m_timIAP = new QTimer(this);
    connect(m_timIAP,SIGNAL(timeout()),this, SLOT(slot_timIAP()));

    //连续规划
    m_sendData = new QTimer(this);
    m_sendData->setInterval(30);
    connect(m_sendData,SIGNAL(timeout()),this, SLOT(slot_timSendData()));

    //m_devId = ui->ledComID->text().toInt();

    //消息提示单例
    AUTOTIP->setParent(this);//tip提示

    //波形显示定时器
    m_plotTimer = new QTimer();
    m_plotTimer->setInterval(50);
    initCustomPlot();

    m_deviceManager = new DeviceManager();
    m_deviceManager->start();
    connect(m_deviceManager, SIGNAL(sensorCtrol(QByteArray)), &serial, SLOT(SendCmdData(QByteArray)));
    connect(m_deviceManager, SIGNAL(sendBack(QByteArray)), this, SLOT(slot_sendBack(QByteArray)));
    this->initSlot();

    ui->buildTIme->setText(QString("Build Time: %1 %2").arg(__DATE__).arg(__TIME__));
    ui->buildTIme->setStyleSheet("font-size:14px");

    ui->btnGetID->hide();
    ui->btnGetPosLimit->hide();
    ui->btnGetCurrentState->hide();
    ui->btnGetCurrentPos->hide();
    ui->btnCalibration->hide();
    ui->btnMotorIDSet->hide();
    ui->m_DispDate->hide();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initSlot()
{
    QList<QPushButton *> btnList;
    btnList.clear();
    btnList  << ui->btnSave << ui->btnSetID << ui->btnClearErr << ui->btnSetPosLimit << ui->btnGetPosLimit\
             << ui->btnServoPosOnce << ui->btnServoPosContiue << ui->btnServoLose << ui->btnServoStop \
             << ui->btnSetCurrentPos << ui->btnGetCurrentPos << ui->btnGetCurrentState \
             << ui->btnParamDefault << ui->btnSetBaud << ui->btnSeParamForce << ui->btnSeParamSpeed \
             << ui->btnGetID << ui->btnGetSystemParam << ui->btnGetMotorState << ui->btnSetZero\
             << ui->btnMoveClose \
             << ui->btnDIOMode << ui->btnDIOOut << ui->btnGetDIOState <<  ui->btnMbSet << ui->btnCalibration\
             << ui->btnMotorIDSet;
    for(int i = 0; i < btnList.count(); i++) {
        connect(btnList.at(i), SIGNAL(clicked()), this, SLOT(slot_btnParamClick()));
    }

    btnList.clear();
    btnList  << ui->btnFilePath << ui->btnStartUpgrade;
    for(int i = 0; i < btnList.count(); i++) {
        connect(btnList.at(i), SIGNAL(clicked()), this, SLOT(slot_btnIAPClick()));
    }

    //曲线显示
    connect(ui->rdbCurrentReal, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));
    connect(ui->rdbSpeedReal, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));
    connect(ui->rdbPosReal, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));
    connect(ui->rdbWaveDivV, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));
    connect(ui->rdbWaveDivH, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));
    connect(ui->rdbWaveRoll, SIGNAL(stateChanged(int)), this, SLOT(slot_rdbWave(int)));

    connect(ui->btnWaveOrigin, SIGNAL(clicked()), this, SLOT(slot_btnWave()));
    connect(ui->btnStartWave, SIGNAL(clicked()), this, SLOT(slot_btnWave()));
    connect(ui->btnWaveClearImage, SIGNAL(clicked()), this, SLOT(slot_btnWave()));

    connect(m_plotTimer, SIGNAL(timeout()), this, SLOT(slot_plotTimer())); //波形显示定时器
}

void MainWindow::slot_btnParamClick()
{
    if(m_connectState != 2) {
        AUTOTIP->SetMesseage(TIPERROR, QString(tr("请先连接设备")));
        return;
    }

    quint16 sendLen;
    QByteArray sendData;
    QObject *sender = QObject::sender();
    if (sender) {
        if(m_protocol == 0) {
            if (sender == ui->btnSave) { //保存参数
                m_deviceManager->sendData(m_devId, CMD_MC_PARA_SAVE, sendData, 0);
            } else if (sender == ui->btnParamDefault) { //恢复默认参数
                m_deviceManager->sendData(m_devId, CMD_MC_PARA_DEFAULT, sendData, 0);
            } else if (sender == ui->btnSetID) { //设置ID
                quint16 id = ui->ledDevID->text().toInt();
                if(id < 1 || id > 255) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("ID设置失败，ID范围1-255")));
                    return;
                }
                sendLen = 1;
                sendData.resize(sendLen);
                sendData[0] = id;
                m_deviceManager->sendData(m_devId, CMD_MC_PARA_ID_SET, sendData, sendLen);
            } else if (sender == ui->btnGetID) { //读取ID
                m_deviceManager->sendData(0xFF, CMD_MC_PARA_ID_GET, sendData, 0);
            } else if (sender == ui->btnSetBaud) { //设置波特率
                quint16 baud = ui->cbxDevBaud->currentIndex();
                if(baud < 0 || baud > 5) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("波特率设置失败，范围0-5")));
                    return;
                }
                sendLen = 1;
                sendData.resize(sendLen);
                sendData[0] = baud;
                m_deviceManager->sendData(m_devId, CMD_MC_PARA_BAUD_SET, sendData, sendLen);
            } else if (sender == ui->btnServoPosOnce) { //以设置的力和速度单次夹取
                sendLen = 4;
                sendData.resize(sendLen);
                quint16 force = ui->ledServoForce->text().toInt();
                quint16 speed = ui->ledServoSpeed->text().toInt();
                if(force < RM_FORCE_MIN || force > RM_FORCE_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围50-1000")));
                    return;
                }
                if(speed < RM_SPEED_MIN || speed > RM_SPEED_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围0-1000")));
                    return;
                }
                sendData[0] = LO_UINT16(speed);
                sendData[1] = HI_UINT16(speed);
                sendData[2] = LO_UINT16(force);
                sendData[3] = HI_UINT16(force);
                m_deviceManager->sendData(m_devId, CMD_MC_MOVE_CATCH_XG, sendData, sendLen);
            } else if (sender == ui->btnServoPosContiue) { //以设置的力和速度持续夹取
                sendLen = 4;
                sendData.resize(sendLen);
                quint16 force = ui->ledServoForce->text().toInt();
                quint16 speed = ui->ledServoSpeed->text().toInt();
                if(force < RM_FORCE_MIN || force > RM_FORCE_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围50-1000")));
                    return;
                }
                if(speed < RM_SPEED_MIN || speed > RM_SPEED_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，速度范围1-1000")));
                    return;
                }
                sendData[0] = LO_UINT16(speed);
                sendData[1] = HI_UINT16(speed);
                sendData[2] = LO_UINT16(force);
                sendData[3] = HI_UINT16(force);
                m_deviceManager->sendData(m_devId, CMD_MC_MOVE_CATCH2_XG, sendData, sendLen);
            } else if (sender == ui->btnServoLose) { //以设置的速度松开
               sendLen = 2;
               sendData.resize(sendLen);
               quint16 speed = ui->ledServoSpeed->text().toInt();
               if(speed < RM_SPEED_MIN || speed > RM_SPEED_MAX) {
                   AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，速度范围0-100")));
                   return;
               }
               sendData[0] = LO_UINT16(speed);
               sendData[1] = HI_UINT16(speed);
               m_deviceManager->sendData(m_devId, CMD_MC_MOVE_RELEASE, sendData, sendLen);
            } else if (sender == ui->btnClearErr) { //清除错误
               m_deviceManager->sendData(m_devId, CMD_MC_ERROR_CLR, sendData, 0);
            } else if (sender == ui->btnSetPosLimit) { //设置开口度限位
                sendLen = 4;
                sendData.resize(sendLen);
                quint16 posMin = ui->ledPosMin->text().toInt();
                quint16 posMax = ui->ledPosMax->text().toInt();
                qDebug() << posMin << posMax;
                if(posMin < RM_POS_MIN || posMin > RM_POS_MAX || \
                   posMax < RM_POS_MIN || posMax > RM_POS_MAX || \
                   posMax <= posMin) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("设置开口度失败，ID范围0-1000")));
                    return;
                }
                sendData[0] = LO_UINT16(posMax);
                sendData[1] = HI_UINT16(posMax);
                sendData[2] = LO_UINT16(posMin);
                sendData[3] = HI_UINT16(posMin);
                m_deviceManager->sendData(m_devId, CMD_MC_SET_EG_PARA, sendData, sendLen);
            } else if (sender == ui->btnGetPosLimit) { //读取开口度限位
                m_deviceManager->sendData(m_devId, CMD_MC_READ_EG_PARA, sendData, 0);
            } else if (sender == ui->btnServoStop) { //急停
                m_deviceManager->sendData(m_devId, CMD_MC_MOVE_STOPHERE, sendData, 0);
            } else if (sender == ui->btnSetCurrentPos) { //控制开口度
                sendLen = 2;
                sendData.resize(sendLen);
                quint16 pos = ui->ledServoPos->text().toInt();
                if(pos < RM_POS_MIN || pos > RM_POS_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，开口度范围0-1000")));
                    return;
                }
                sendData[0] = LO_UINT16(pos);
                sendData[1] = HI_UINT16(pos);
                m_deviceManager->sendData(m_devId, CMD_MC_SEEKPOS, sendData, sendLen);
            } else if (sender == ui->btnGetCurrentPos) { //读取当前开口度
                m_deviceManager->sendData(m_devId, CMD_MC_READ_ACTPOS, sendData, 0);
            } else if (sender == ui->btnGetCurrentState) { //读取当前状态
                m_deviceManager->sendData(m_devId, CMD_MC_READ_EG_RUNSTATE, sendData, 0);
            } else if (sender == ui->btnSeParamForce) { //设置力参数
                sendLen = 2;
                sendData.resize(sendLen);
                quint16 forceMax = ui->ledServoForce->text().toInt();
                if(forceMax > RM_FORCE_MAX) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("设置力最大值失败，范围****")));
                    return;
                }
                sendData[0] = LO_UINT16(forceMax);
                sendData[1] = HI_UINT16(forceMax);
                m_deviceManager->sendData(m_devId, CMD_MC_SET_FORCE_PARA, sendData, sendLen);
            } else if (sender == ui->btnSeParamSpeed) { //设置速度参数
                quint16 speed = ui->ledServoSpeed->text().toInt();
                if(speed < RM_SPEED_MIN || speed > RM_SPEED_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围0-1000")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_SET_SPEED, speed);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnGetSystemParam) { //读取当前状态
                m_deviceManager->sendData(m_devId, CMD_MC_READ_SYSTEM_PARAM, sendData, 0);
            } else if (sender == ui->btnGetMotorState) { //读取当前状态
                m_deviceManager->sendData(m_devId, CMD_MC_READ_MOTOR_STATE, sendData, 0);
            } else if (sender == ui->btnSetZero) { //设置零位
                m_deviceManager->sendData(m_devId, CMD_MC_SET_ZERO, sendData, 0);
            } else if (sender == ui->btnMoveClose) { //单步运动
                sendLen = 4;
                float pos = ui->ledOneStep->text().toFloat();
                if(pos>10 || pos <-10) {
                    AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意单步角度小于10度")));
                    return;
                }
                sendData.resize(sendLen);
                memcpy(sendData.data(), &pos, sizeof(float));
                m_deviceManager->sendData(m_devId, CMD_MC_SERVO_MOVE, sendData, sendLen);
            } else if (sender == ui->btnDIOMode) { //IO模式设置
                sendLen = 2;
                sendData.resize(sendLen);
                sendData[1] = ui->cbxDIOMode->currentIndex();
                sendData[0] = ui->cbxDIONum->currentIndex();
                m_deviceManager->sendData(m_devId, CMD_MC_SET_DIO_MODE, sendData, sendLen);
            } else if (sender == ui->btnDIOOut) { //IO输出
                sendLen = 2;
                sendData.resize(sendLen);
                sendData[1] = ui->cbxDIOOut->currentIndex();
                sendData[0] = ui->cbxDIONum->currentIndex();
                m_deviceManager->sendData(m_devId, CMD_MC_SET_DIO_OUT, sendData, sendLen);
            } else if (sender == ui->btnGetDIOState) { //IO输出
                m_deviceManager->sendData(m_devId, CMD_MC_GET_DIO_STATE, sendData, 0);
            } else if (sender == ui->btnCalibration) { //校准
                //m_deviceManager->sendData(m_devId, CMD_MC_GET_DIO_STATE, sendData, 0);
                ui->horizontalSlider->setValue(postionReal);
            } else if (sender == ui->btnMotorIDSet) { //设置关节ID
                quint16 id = 1;
                sendLen = 1;
                sendData.resize(sendLen);
                sendData[0] = id;
                m_deviceManager->sendData(m_devId, CMD_MC_MOTOR_ID_SET, sendData, sendLen);
            }
        }
        else //RTU协议
        {
            if (sender == ui->btnSave) { //保存参数
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_PARAM_SAVE, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnParamDefault) { //恢复默认参数
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_PARAM_DEFAULT, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSetID) { //设置ID
                quint16 id = ui->ledDevID->text().toInt();
                if(id < 1 || id > 255) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("ID设置失败，ID范围1-255")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_DEV_ID, id);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSetBaud) { //设置波特率
                quint16 baud = ui->cbxDevBaud->currentIndex();
                if(baud < 0 || baud > 5) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("波特率设置失败，范围0-5")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_UART_BAUD, baud);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnServoPosOnce) { //以设置的力和速度单次夹取
                on_cbxPlan_stateChanged(0);
                ui->cbxPlan->setCheckState(Qt::CheckState::Unchecked);
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_CATCH, 0);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnServoPosContiue) { //以设置的力和速度持续夹取
                on_cbxPlan_stateChanged(0);
                ui->cbxPlan->setCheckState(Qt::CheckState::Unchecked);
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_CATCH, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnServoLose) { //以设置的速度松开
                on_cbxPlan_stateChanged(0);
                ui->cbxPlan->setCheckState(Qt::CheckState::Unchecked);
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_LOOSE, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnServoStop) { //急停
                on_cbxPlan_stateChanged(0);
                ui->cbxPlan->setCheckState(Qt::CheckState::Unchecked);
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_STOP, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnClearErr) { //清除错误
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_ERR_CLE, 1);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSetCurrentPos) { //设置开口度
                quint16 pos = ui->ledServoPos->text().toInt();
                if(pos < RM_POS_MIN || pos > RM_POS_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，开口度范围0-1000")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_SET_POS, pos);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSeParamForce) { //设置力参数
                quint16 force = ui->ledServoForce->text().toInt();
                if(force < RM_FORCE_MIN || force > RM_FORCE_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围50-1000")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_SET_FORCE, force);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSeParamSpeed) { //设置速度参数
                quint16 speed = ui->ledServoSpeed->text().toInt();
                if(speed < RM_SPEED_MIN || speed > RM_SPEED_MAX) {
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("错误，力范围0-1000")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_SET_SPEED, speed);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnSetPosLimit) { //设置开口度限位
                quint16 posMin = ui->ledPosMin->text().toInt();
                quint16 posMax = ui->ledPosMax->text().toInt();
                qDebug() << posMin << posMax;
                if(posMin < RM_POS_MIN || posMin > RM_POS_MAX || \
                   posMax < RM_POS_MIN || posMax > RM_POS_MAX || \
                   posMax <= posMin) {
                    //提示错误
                    AUTOTIP->SetMesseage(TIPERROR, QString(tr("设置开口度失败，ID范围0-65")));
                    return;
                }
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_POS_MIN, posMin);
                sensor_manager::getInstance()->send_rtu_data(sendData);
                msleep(800);
                sendData = m_mbCreate.writeSingleRegister(m_devId, MB_REG_POS_MAX, posMax);
                sensor_manager::getInstance()->send_rtu_data(sendData);
            } else if (sender == ui->btnDIOMode) { //IO模式设置
                int io = ui->cbxDIOMode->currentIndex();
                int mode = ui->cbxDIONum->currentIndex();
                if(io == 0) {
                    if(mode == 0) {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_MODE_0, false);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    } else {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_MODE_0, true);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    }
                } else if(io == 1) {
                    if(mode == 0) {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_MODE_1, false);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    } else {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_MODE_1, true);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    }
                }
            } else if (sender == ui->btnDIOOut) { //IO输出
                int io = ui->cbxDIOMode->currentIndex();
                int out = ui->cbxDIONum->currentIndex();
                if(io == 0) {
                    if(out == 0) {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_STATE_0, false);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    } else {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_STATE_0, true);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    }
                } else if(io == 1) {
                    if(out == 0) {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_STATE_1, false);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    } else {
                        sendData = m_mbCreate.writeSingleCoil(m_devId, MB_COILS_DIO_STATE_1, true);
                        sensor_manager::getInstance()->send_rtu_data(sendData);
                    }
                }
            } else if (sender == ui->btnMbSet) { //IO输出
                //int regType = ui->cbxMbReg->currentIndex();
                int func = ui->cbxMbFunc->currentIndex()+1;
                int addr = ui->ledMbAddr->text().toInt();
                int value = ui->ledMbValue->text().toInt();

                if(func == 1) {
                    sendData = m_mbCreate.readCoils(m_devId, addr, 1);
                    sensor_manager::getInstance()->send_rtu_data(sendData);
                } else if(func == 3) {
                    sendData = m_mbCreate.readHoldingRegisters(m_devId, addr, 1);
                    sensor_manager::getInstance()->send_rtu_data(sendData);
                } else if(func == 5) {
                    sendData = m_mbCreate.writeSingleCoil(m_devId, addr, bool(value));
                    sensor_manager::getInstance()->send_rtu_data(sendData);
                } else if(func == 6) {
                    sendData = m_mbCreate.writeSingleRegister(m_devId, addr, quint16(value));
                    sensor_manager::getInstance()->send_rtu_data(sendData);
                }
            }
         }
    }
}

void MainWindow::iapStateInit(void)
{
    m_iapStartFlag = false;
    m_iapStartCount = 0;
    m_iapFileSize = 0;
    m_iapOffset = 0;
    m_iapEnable = 0;
    m_iapSquence = 0;
    m_iapTimeout = 0;
    m_iapFile.close();
    m_timIAP->stop();
    ui->pgbUpgrade->setValue(0);
}

void MainWindow::cmd_485_data_process(quint8 _cmd, QByteArray _data)
{
    QString strTmp;
    quint8 id = _data[2];
    quint16 sendLen;
    QByteArray sendData;

    //_cmd == CMD_MC_SEEKPOS || \

    if(_cmd == CMD_MC_PARA_SAVE || \
       _cmd == CMD_MC_PARA_DEFAULT || \
       _cmd == CMD_MC_PARA_BAUD_SET || \
       _cmd == CMD_MC_PARA_ID_SET || \
       _cmd == CMD_MC_MOVE_CATCH_XG || \
       _cmd == CMD_MC_MOVE_CATCH2_XG || \
       _cmd == CMD_MC_MOVE_RELEASE || \
       _cmd == CMD_MC_MOVE_STOPHERE || \
       _cmd == CMD_MC_ERROR_CLR || \
       _cmd == CMD_MC_SET_EG_PARA || \
       _cmd == CMD_MC_SERVO_MOVE || \
       _cmd == CMD_MC_SET_DIO_MODE || \
       _cmd == CMD_MC_SET_DIO_OUT || \
       _cmd == CMD_MC_SET_FORCE_PARA || \
       _cmd == CMD_MC_SET_KEY_ENABLE || \
       _cmd == CMD_MC_MOTOR_ID_SET) {
        if((quint8)_data[5] == CMD_MC_OK) {
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("成功：%1")).arg(_cmd));
        } else {
            if((quint8)_data[5] == CMD_MC_OUT_RANGE) {
                AUTOTIP->SetMesseage(TIPERROR, QString(tr("超限位错误：%1")).arg(_cmd));
            } else if((quint8)_data[5] == CMD_MC_RUNING) {
                AUTOTIP->SetMesseage(TIPERROR, QString(tr("设备运行中：%1")).arg(_cmd));
            } else {
                AUTOTIP->SetMesseage(TIPERROR, QString(tr("失败：%1")).arg(_cmd));
            }
        }
    } else if(_cmd == CMD_MC_PARA_ID_GET) {
        quint8 ID = (quint8)_data[5];
        ui->labCurrentDevId->setNum(ID);
        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取当前ID成功")));
    } else if(_cmd == CMD_MC_READ_EG_PARA) {
        quint16 posMax = BUILD_UINT16((quint8)_data[5], (quint8)_data[6]);
        quint16 posMin = BUILD_UINT16((quint8)_data[7], (quint8)_data[8]);
        ui->labPosMax->setNum(posMax);
        ui->labPosMin->setNum(posMin);
        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取开口度范围成功")));
    } else if(_cmd == CMD_MC_GET_DIO_STATE) {
        quint8 dio0Mode = (quint8)_data[5];
        quint8 dio0State = (quint8)_data[6];
        quint8 dio1Mode = (quint8)_data[7];
        quint8 dio1State = (quint8)_data[8];
        QString modeStr = (dio0Mode == 0) ? "输入模式" : "输出模式";
        QString stateStr = (dio0State == 0) ? "低电平" : "高电平";
        QString strDio = QString("%1, %2").arg(modeStr).arg(stateStr);
        ui->labDIO0State->setText(strDio);
        modeStr = (dio1Mode == 0) ? "输入模式" : "输出模式";
        stateStr = (dio1State == 0) ? "低电平" : "高电平";
        strDio = QString("%1, %2").arg(modeStr).arg(stateStr);
        ui->labDIO1State->setText(strDio);
        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取IO状态成功")));
    } else if(_cmd == CMD_MC_READ_ACTPOS) {
        quint16 posCurrent = BUILD_UINT16((quint8)_data[5], (quint8)_data[6]);
        ui->labCurrentActpos->setNum(posCurrent);

        quint8 mm = RM_RANGE_MAX*(posCurrent/1000.0);
        ui->labCurrentActpos_2->setNum(mm);


        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取当前开口度成功")));
    } else if(_cmd == CMD_MC_GET_FORCE_PARA) {
        quint16 force = BUILD_UINT16((quint8)_data[5], (quint8)_data[6]);
        ui->labServoForce->setNum(force);
        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取当前力度成功")));
    } else if(_cmd == CMD_MC_READ_SYSTEM_PARAM) {
        quint8 id = (quint8)_data[5];
        quint8 baud = (quint8)_data[6];

        quint16 pos_limit_min = BUILD_UINT16((quint8)_data[7],(quint8)_data[8]);
        quint16 pos_limit_max = BUILD_UINT16((quint8)_data[9],(quint8)_data[10]);
        quint16 servo_speed = BUILD_UINT16((quint8)_data[11],(quint8)_data[12]);

        quint16 force = BUILD_UINT16((quint8)_data[13],(quint8)_data[14]);
        quint16 version = BUILD_UINT16((quint8)_data[15],(quint8)_data[16]);

        ui->labCurrentDevId->setNum(id);
        ui->labBaud->setText(m_listBaud.at(baud));
        ui->labPosMin->setNum(pos_limit_min);
        ui->labPosMax->setNum(pos_limit_max);
        ui->labServoSpeed->setNum(servo_speed);
        ui->labServoForce->setNum(force);
        ui->labVersion->setText(convertVersion(version));

        AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取系统参数成功")));
    } else if(_cmd == CMD_MC_READ_MOTOR_STATE) {
        quint8 en = (quint8)_data[5];
        quint8 run_state = (quint8)_data[6];
        quint16 error = BUILD_UINT16((quint8)_data[7], (quint8)_data[8]);
        currentReal = byteToFloat(_data.mid(9, 4));
        voltageReal = byteToFloat(_data.mid(13, 4));
        angleReal = byteToFloat(_data.mid(17, 4));
        speedReal = byteToFloat(_data.mid(21, 4));
        tempReal = byteToFloat(_data.mid(25, 4));
        float force = byteToFloat(_data.mid(29, 4));
        postionReal = BUILD_UINT16((quint8)_data[33], (quint8)_data[34]);
        quint8 sys_error = (quint8)_data[35];

        ui->labCurrentActpos->setText(QString::number(postionReal));
        quint16 mm = RM_RANGE_MAX*(postionReal/1000.0);
        ui->labCurrentActpos_2->setNum(mm);
        ui->labCurrentForce->setText(QString::number(force, 'f', 1));
        ui->labCurrentCurrent->setText(QString::number(currentReal, 'f', 3));
        ui->labCurrentVoltage->setText(QString::number(voltageReal, 'f', 3));
        ui->labCurrentPos->setText(QString::number(angleReal, 'f', 3));
        ui->labCurrentSpeed->setText(QString::number(speedReal, 'f', 3));
        ui->labCurrentTemp->setText(QString::number(tempReal, 'f', 3));
        //ui->labError->setNum(error);
        ui->labCurrentState->setText(m_listState.at(run_state-1));
        ui->labEnState->setNum(en);
        ui->labEnState->setText(en ? tr("上使能"):tr("掉使能"));

        //电机错误信息
        QString strErrorInfo = " ";
        if((uint8_t)sys_error & (0x01<<0)) {
            strErrorInfo = tr("堵转错误");
        } else if((uint8_t)sys_error & (0x01<<1)) {
            strErrorInfo = tr("过温错误");
        } else if((uint8_t)sys_error & (0x01<<2)) {
            strErrorInfo = tr("过流错误");
        } else if((uint8_t)sys_error & (0x01<<3)) {
            strErrorInfo = tr("驱动器故障");
        } else if((uint8_t)sys_error & (0x01<<4)) {
            strErrorInfo = tr("内部通讯错误");
        } else if((uint8_t)sys_error & (0x01<<5)) {
            QMap<quint16, QString>::iterator itErr = m_errorMap.begin();
            while (itErr != m_errorMap.end()) {
                if(itErr.key()&error) {
                    strErrorInfo += itErr.value();
                }
                ++itErr;
            }
        }
        ui->labError->setText(strErrorInfo);
    } else if(_cmd == CMD_MC_READ_EG_RUNSTATE) {
        //quint8 state = (quint8)_data[5];
//        quint8 error = (quint8)_data[6];
//        quint8 temp = (quint8)_data[7];
//        quint16 pos = BUILD_UINT16((quint8)_data[8], (quint8)_data[9]);
//        quint16 force = BUILD_UINT16((quint8)_data[10], (quint8)_data[11]);
//        quint16 speed = BUILD_UINT16((quint8)_data[12], (quint8)_data[13]);
//        quint16 voltage = BUILD_UINT16((quint8)_data[14], (quint8)_data[15]);
//        quint16 current = BUILD_UINT16((quint8)_data[16], (quint8)_data[17]);

        //ui->labCurrentState_2->setNum(state);
        //ui->labCurrentState_2->setText(m_listState.at(state-1));
//        ui->labError->setNum(error);
//        ui->labCurrentTemp->setNum(temp);
//        ui->labCurrentPos->setNum(pos);
//        ui->labCurrentForce->setNum(force);
//        ui->labCurrentSpeed->setNum(speed);
//        ui->labCurrentVoltage->setNum(voltage);
//        ui->labCurrentCurrent->setNum(current);
        //AUTOTIP->SetMesseage(TIPINFO, QString(tr("读取当前状态成功")));
    } else if(_cmd == CMD_MC_UPGRADE) {
        quint8 device = (quint8)_data[5];
        quint8 index = (quint8)_data[6];
        //quint16 squence = BUILD_UINT16((quint8)_data[7], (quint8)_data[8]);

        if(index == E_UPGRADE_IMG_REQ) { //请求升级包 2
            m_iapStartFlag = false;
            m_iapStartCount = 0;
            if(device != m_iapType) {
                iapStateInit();
                AUTOTIP->SetMesseage(TIPWARRING, QString(tr("升级请求不匹配")));
            }
            if(m_iapOffset >= m_iapFileSize) {
                return; //过滤最后一帧
            }
            m_iapTimeout = 30; //3s
            quint8 buf[64] = {0xFF};
            int len = m_iapFile.read((char *)buf, 64);
            m_iapOffset += len;
            qDebug() << "offset" << m_iapOffset << m_iapFileSize;
            m_iapSquence++;
            if(len > 0) {
                int bar = (m_iapOffset*100)/m_iapFileSize;
                ui->pgbUpgrade->setValue(bar);
                sendLen = 4 + 64;
                sendData.resize(sendLen);
                sendData[0] = m_iapType; //设备ID
                sendData[1] = E_UPGRADE_IMG_REQ; //升级包 2
                sendData[2] = (quint8)m_iapSquence;
                sendData[3] = (quint8)(m_iapSquence>>8);
                for (int i = 0; i < len; i++) {
                    sendData[4 + i] = buf[i];
                }
                //msleep(10);
                m_deviceManager->sendData(m_devId, CMD_MC_UPGRADE, sendData, sendLen);

                if(m_iapOffset >= m_iapFileSize) { //最后一帧
                    msleep(10);
                    m_iapFile.close();
                    sendLen = 4;
                    sendData.resize(sendLen);
                    sendData[0] = m_iapType; //设备ID
                    sendData[1] = E_UPGRADE_FINISH; //升级完成 3
                    sendData[2] = (quint8)m_iapSquence;
                    sendData[3] = (quint8)(m_iapSquence>>8);
                    m_deviceManager->sendData(m_devId, CMD_MC_UPGRADE, sendData, sendLen);
                    qDebug() << "最后一帧";
                    return;
                }
            }
        } else if(index == E_UPGRADE_FINISH) { //升级完成 3
            iapStateInit();
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("升级成功")));
        } else if(index == E_UPGRADE_FAILED) { //升级失败 4
            iapStateInit();
            AUTOTIP->SetMesseage(TIPWARRING, QString(tr("升级失败")));
        } else if(index == E_UPGRADE_RESTART) { //再次请求升级 5
            if(device == 2) {
                sendLen = 4;
                sendData.resize(sendLen);
                sendData[0] = m_iapType; //设备ID
                sendData[1] = E_UPGRADE_AGREE_RESTART; //开始升级 6
                sendData[2] = (quint8)m_iapSquence;
                sendData[3] = (quint8)(m_iapSquence>>8);
                m_deviceManager->sendData(m_devId, CMD_MC_UPGRADE, sendData, sendLen);
            }
        } else if(index == E_UPGRADE_AGREE_RESTART) { //同意再次升级请求 6
        }
    }
}

//IAP相关
void MainWindow::slot_timIAP()
{
    m_iapTimeout--;
    if(m_iapTimeout <= 0){
        AUTOTIP->SetMesseage(TIPWARRING, QString("注意：超时退出升级"));
        //m_timIAP->stop();
        //m_iapFile.close();
        iapStateInit();
    }
}

void MainWindow::slot_btnIAPClick()
{
    quint16 sendLen = 0;
    QByteArray sendData;

    QObject *sender = QObject::sender();
    if (sender) {
        if (sender == ui->btnFilePath) { //获取文件
            QString name;
            name = QFileDialog::getOpenFileName(NULL, tr("IAP文件"), name, "*.bin");
            ui->ledFiePath->setText(name);
            ui->pgbUpgrade->setValue(0);
        } else if (sender == ui->btnStartUpgrade) { //开始升级
            m_iapType = ui->cbxUpgradeType->currentIndex();
            QString path = ui->ledFiePath->text();
            if(m_iapType == 1) { //控制板升级
                if(!path.contains("gripper_app")) {
                    AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：请选择正确的app固件")));
                    return ;
                }
            } else if(m_iapType == 2) { //电机升级
//                if(!path.contains("RD03")) {
//                    AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：请选择正确的关节固件")));
//                    return ;
//                }
            } else {
                AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：请选择要升级的设备")));
                return ;
            }
            if(path.isEmpty()) {
                AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：请选择升级文件")));
                return ;
            }
            m_iapOffset = 0; //偏移计数
            m_iapEnable = 0; //升级使能
            m_iapSquence = 0; //序列号
            m_iapTimeout = 0; //超时时间
            m_sendData->stop();
            m_plotTimer->stop();
            ui->pgbUpgrade->setValue(0);

            m_iapFile.setFileName(path);
            if(!m_iapFile.open(QIODevice::ReadOnly)) {
                AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：升级文件校验失败")));
                return;
            }
            if(m_iapFile.size() < 2000) {
                AUTOTIP->SetMesseage(TIPWARRING, QString("注意：升级文件校验失败"));
                m_iapFile.close();
                return;
            }
            m_iapFileSize = m_iapFile.size();
            qDebug() << "file size >> " << m_iapFileSize;
            sendLen = 4;
            sendData.resize(sendLen);
            sendData[0] = m_iapType; //设备ID
            sendData[1] = E_UPGRADE_SART; //开始升级 1
            sendData[2] = 0;
            sendData[3] = 0;
            m_deviceManager->sendData(m_devId, CMD_MC_UPGRADE, sendData, sendLen);
            m_iapTimeout = 100; //10
            m_iapStartFlag = true;
            m_iapStartCount = 3;
            //AUTOTIP->SetMesseage(TIPINFO, QString("注意：请在10秒内重启设备"));
            m_timIAP->start(100);
        }
    }
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    m_iapEnable = arg1;

    if(arg1) {
        ui->btnStartUpgrade->setEnabled(true);
    } else {
        ui->btnStartUpgrade->setEnabled(false);
    }
}

void MainWindow::cmd_rtu_data_process(quint8 _func, QByteArray _data)
{
    if(((unsigned char)_func & 0x80) == 0) { //正确
        int addr = 0, value = 0;

        if(_func == E_READ_COILS)
        {
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("读线圈成功，功能码：%1 地址：%2")).arg(_func).arg(addr));
            if((quint8)_data[3] == 1) {
                value = (quint8)_data[3];
                ui->ledMbValue->setText(QString("%1").arg(value));
            }
        }
        else if(_func == E_READ_HOLDING)
        {
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("读保持寄存器成功，功能码：%1 地址：%2")).arg(_func).arg(addr));
            value = BUILD_UINT16((quint8)_data[4], (quint8)_data[3]);
            ui->ledMbValue->setText(QString("%1").arg(value));
        }
        else if(_func == E_WRITE_SINGLE_COIL)
        {
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("写线圈成功，功能码：%1 地址：%2")).arg(_func).arg(addr));
        }
        else if(_func == E_WRITE_SINGLE_HOLDING)
        {
            addr = BUILD_UINT16((quint8)_data[3], (quint8)_data[2]);
            AUTOTIP->SetMesseage(TIPINFO, QString(tr("写保持寄存器成功，功能码：%1 地址：%2")).arg(_func).arg(addr));
        }
    } else { //错误
        unsigned char efunc = ((unsigned char)_func & 0x7F);
        unsigned char eCode = _data[2];
        AUTOTIP->SetMesseage(TIPERROR, QString(tr("RTU响应错误，功能码：%1 错误码：%2")).arg(efunc).arg(eCode));
    }
}

/* 显示日期 */
void MainWindow::DispDate_Slot(void)
{
    QDateTime dateTime(QDateTime::currentDateTime());
    ui->m_DispDate->setText(dateTime.toString("yyyy-MM-dd hh:mm:ss ddd"));

    if(m_iapStartFlag == true)
    {
        if(m_iapStartCount <= 0) {
            m_iapStartFlag = false;
            //AUTOTIP->SetMesseage(TIPINFO, QString("注意：请在10秒内重启设备"));
        } else {
            m_iapStartCount --;
        }
    }

    if(m_connectState != 2) {
        emit sig_findDev();
    }

    //updateDeviceList();

}
/* 串口数据接收 */
void MainWindow::RecvSensorData(QByteArray _bSensorData)
{
    online_timeout = 20;
    //sensor_manager::getInstance()->save_dev_info(_bSensorData); //传感器保存设备列表
    if((quint8)_bSensorData.at(0) == 0xEE && (quint8)_bSensorData.at(1) == 0x16 ) {
        cmd_485_data_process((quint8)_bSensorData.at(4), _bSensorData); //解析传感器数据
    } else {
        // 从站地址1 功能码1 寄存器地址2 寄存器值2 CRC校验2
        // 从站地址1 功能码1 错误码1 CRC校验2
        cmd_rtu_data_process((quint8)_bSensorData.at(1), _bSensorData); //解析传感器数据
    }
    QString hexString = byteArrayToHexString(_bSensorData);
    ui->ledRecvData->setText(hexString);
    ui->labRecvLen->setNum(_bSensorData.size());
}

/* 串口状态显示 */
void MainWindow::DispConnectState_Slot(int state)
{
    m_connectState = state;

    switch (state)
    {
    case 0: //Disconnected
        ui->connect_btn->setText("打开");
        break;
    case 1:
         ui->connect_btn->setText("正在连接");
        break;
    case 2:

        ui->connect_btn->setText("关闭");
        break;
    case 3:
//        QMessageBox::information(NULL, "串口配置", "串口打开失败,请检查是否被占用，或重新插拔串口后重启软件重试",
//                                 QMessageBox::Yes);
        //ui->connect_btn->setText("打开");
        break;
    default:
        break;
    }
}
void MainWindow::on_connect_btn_clicked()
{
    QString name = comBox->currentText();
    int baud = ui->baudrate->currentText().toInt();
    if(ui->connect_btn->text() == "打开")
    {
        emit sig_openSerial(name, baud);
    }
    else
    {
        emit CloseUartDev_Signal(); //断开串口信号
    }
}

int MainWindow::updateDeviceList()
{
    int count  = comBox->count();
    while(comBox->count())
    {
        comBox->removeItem(0);
    }
    //serial.findSerialDevices();
    comBox->addItems(serial.portsList());
    return count;
}

quint8 MainWindow::sum_check(quint8 *data, quint16 len)
{
    quint32 sum = 0;
    for(int i = 0; i < len; i++)
    {
        sum += data[i];
    }

    return sum&0x000000FF;
}
QString MainWindow::byteArrayToHexString(const QByteArray &data) {
    QString hexString;
    for (char byte : data) {
        // 使用 QString::sprintf 格式化字节为 16 进制
        hexString.append(QString("%1 ").arg(static_cast<unsigned char>(byte), 2, 16, QLatin1Char('0')).toUpper());
    }
    // 去除最后一个多余的空格
    if (!hexString.isEmpty()) {
        hexString.chop(1);
    }
    return hexString;
}

void MainWindow::slot_sendBack(QByteArray _bSensorData)
{
    QString hexString = byteArrayToHexString(_bSensorData);
    ui->ledSendData->setText(hexString);
    ui->labSendLen->setNum(_bSensorData.size());
}

void MainWindow::on_cbxProtocol_currentIndexChanged(int index)
{
    m_protocol = index;
}
void MainWindow::msleep(int msec)
{
#if 0
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
#else
    QEventLoop loop;
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    timer.setSingleShot(true);
    timer.start(msec);
    loop.exec();
#endif
}

void MainWindow::on_spbxComID_valueChanged(int arg1)
{
    if(arg1 < 1 || arg1 > 255){
        AUTOTIP->SetMesseage(TIPERROR, QString(tr("ID范围1~255")));
        return;
    }
    m_devId = arg1;
}
QString MainWindow::convertVersion(int version) {
    int major = version / 100;
    int minor = (version % 100) / 10;
    int patch = version % 100;

    return QString("V%1.%2.%3").arg(major).arg(minor).arg(patch);
}
float MainWindow::byteToFloat(QByteArray data)
{
    quint8 byte0 = (quint8)data.at(0);  // 低字节
    quint8 byte1 = (quint8)data.at(1);
    quint8 byte2 = (quint8)data.at(2);
    quint8 byte3 = (quint8)data.at(3);  // 高字节

    // 将四个字节组合成一个32位整数
    quint32 combined = (static_cast<quint32>(byte3) << 24) |
                       (static_cast<quint32>(byte2) << 16) |
                       (static_cast<quint32>(byte1) << 8)  |
                       (static_cast<quint32>(byte0));

    // 将32位整数转换为浮点数
    float result = 0;
    memcpy(&result, &combined, sizeof(result));

    return result;
}

void MainWindow::slot_timSendData()
{
    quint16 speed = 1000, force = 1000;
    quint16 value = ui->horizontalSlider->value();
    QByteArray sendData;
    sendData.resize(6);
    quint16 sendLen = 6;
    sendData[0] = LO_UINT16(value);
    sendData[1] = HI_UINT16(value);
    sendData[2] = LO_UINT16(speed);
    sendData[3] = HI_UINT16(speed);
    sendData[4] = LO_UINT16(force);
    sendData[5] = HI_UINT16(force);
    //SERIAL->send_485_data(m_devId, CMD_MC_SEEKPOS_SPEED_FORCE, sendData, sendLen);
    m_deviceManager->sendData(m_devId, CMD_MC_SEEKPOS_SPEED_FORCE, sendData, sendLen);
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    qDebug() << value;
}

void MainWindow::initCustomPlot(void)
{
    // 添加曲线
    m_graphCurrentReal = ui->customPlot->addGraph();
    m_graphSpeedReal = ui->customPlot->addGraph();
    m_graphPosReal = ui->customPlot->addGraph();

    //设置抗锯齿
    m_graphCurrentReal->setAntialiasedFill(true);
    // 设置曲线颜色
    QPen pen;
    pen.setWidth(2);//曲线的粗细
    pen.setColor(Qt::green);
    m_graphCurrentReal->setPen(pen);
    pen.setColor(Qt::red);
    m_graphSpeedReal->setPen(pen);
    pen.setColor(Qt::blue);
    m_graphPosReal->setPen(pen);
    // 添加曲线名称
    m_graphCurrentReal->setName(tr("实际电流"));
    m_graphSpeedReal->setName(tr("实际速度"));
    m_graphPosReal->setName(tr("实际位置"));
    // 设置坐标轴名称
    ui->customPlot->xAxis->setLabel("Time");
    ui->customPlot->yAxis->setLabel("Data");
    // 设置y坐标轴显示范围
    ui->customPlot->yAxis->setRange(-10000,10000);
    ui->customPlot->xAxis->setRange(0,10000);
    // 显示图表的图例
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setBrush(QColor(238, 238, 238, 0)); //灰色透明
    ui->customPlot->legend->setBorderPen(Qt::NoPen); //隐藏边框
    // 设置背景颜色
    ui->customPlot->setBackground(QBrush(QColor(238, 238, 238)));

    // 鼠标滚动调整
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    //鼠标滚动值调整Y轴，或者XY同时调整Qt::Vertical | Qt::Horizontal
    ui->customPlot->axisRect()->setRangeZoom(Qt::Vertical | Qt::Horizontal);

    //初始不是能曲线
    m_graphCurrentReal->setVisible(false);
    m_graphSpeedReal->setVisible(false);
    m_graphPosReal->setVisible(false);

    //ui->customPlot->yAxis->rescale(true);//自动缩放
    //ui->customPlot->replot();
}

void MainWindow::slot_rdbWave(int state)
{
    QObject *sender = QObject::sender();  // 获取发送者对象
    if (sender) {
        if (sender == ui->rdbCurrentReal) {
            m_waveEnable.currentReal = state;
            if(state) {
                m_graphCurrentReal->setVisible(true);
            } else {
                m_graphCurrentReal->setVisible(false);
            }
        } else if (sender == ui->rdbSpeedReal) {
            m_waveEnable.speedReal = state;
            if(state) {
                m_graphSpeedReal->setVisible(true);
            } else {
                m_graphSpeedReal->setVisible(false);
            }
        } else if (sender == ui->rdbPosReal) {
            m_waveEnable.posReal = state;
            if(state) {
                m_graphPosReal->setVisible(true);
            } else {
                m_graphPosReal->setVisible(false);
            }
        } else if (sender == ui->rdbWaveDivH || sender == ui->rdbWaveDivV) {
            if(ui->rdbWaveDivH->isChecked() && ui->rdbWaveDivV->isChecked()) {
                //XY可调
                ui->customPlot->axisRect()->setRangeZoom(Qt::Vertical | Qt::Horizontal);
            } else if(ui->rdbWaveDivH->isChecked() && !ui->rdbWaveDivV->isChecked()) {
                ui->customPlot->axisRect()->setRangeZoom(Qt::Horizontal);
            } else if(!ui->rdbWaveDivH->isChecked() && ui->rdbWaveDivV->isChecked()) {
                ui->customPlot->axisRect()->setRangeZoom(Qt::Vertical);
            } else {
                ui->customPlot->axisRect()->setRangeZoom(0);
            }
        } else if (sender == ui->rdbWaveRoll) {
            m_waveRoll = state;
        }

        ui->customPlot->replot();
    }
}

void MainWindow::slot_btnWave()
{
    if(m_connectState != 2) {
        AUTOTIP->SetMesseage(TIPERROR, QString(tr("请先连接设备")));
        return;
    }

    QObject *sender = QObject::sender();
    if (sender) {
        if (sender == ui->btnWaveOrigin) {
            //回到原点
            ui->customPlot->yAxis->setRange(-10000,10000);
            ui->customPlot->xAxis->setRange(0,10000);
            ui->customPlot->replot();
        } else if (sender == ui->btnStartWave) {

            if(ui->btnStartWave->text() == "开始") {
                if(m_waveEnable.currentReal == true) {
                    m_graphCurrentReal->setVisible(true); //实际电流
                }
                if(m_waveEnable.speedReal == true) {
                    m_graphSpeedReal->setVisible(true); //实际速度
                }
                if(m_waveEnable.posReal == true) {
                    m_graphPosReal->setVisible(true); //实际位置
                }
                //开始显示
                if(m_waveEnable.posTarget == true || m_waveEnable.posReal == true \
                || m_waveEnable.speedTarget == true ||  m_waveEnable.speedReal == true \
                || m_waveEnable.currentTarget == true || m_waveEnable.currentReal == true) {
                    m_plotTimer->start();
                    ui->btnStartWave->setText(tr("暂停"));
                } else {
                    AUTOTIP->SetMesseage(TIPWARRING, QString(tr("注意：未选择任何曲线")));
                }
            } else {
                //暂停显示
                m_plotTimer->stop();
                ui->btnStartWave->setText(tr("开始"));
            }
        } else if (sender == ui->btnWaveClearImage) {
            //清空
            m_lineCnt = 0; //重新计数
            //回到原点
            ui->customPlot->yAxis->setRange(-1000,1000);
            ui->customPlot->xAxis->setRange(0,1000);
            int count=ui->customPlot->graphCount();//获取曲线条数
            for(int i=0;i<count;++i) {
                ui->customPlot->graph(i)->data().data()->clear();
            }
            ui->customPlot->replot(); //重新绘制
        }
    }
}

void MainWindow::slot_plotTimer()
{
    QByteArray sendData;
    m_deviceManager->sendData(m_devId, CMD_MC_READ_MOTOR_STATE, sendData, 0);

    //实际电流
    if(m_waveEnable.currentReal == true) {
        m_graphCurrentReal->addData(m_lineCnt, currentReal);
    }

    //实际速度
    if(m_waveEnable.speedReal == true) {
        m_graphSpeedReal->addData(m_lineCnt, speedReal);
    }

    //实际位置
    if(m_waveEnable.posReal == true) {
        m_graphPosReal->addData(m_lineCnt, postionReal);
    }

    m_lineCnt++;
    if(m_waveRoll) { //超过1000滚动显示
        ui->customPlot->xAxis->setRange((m_lineCnt>500)?(m_lineCnt-500):0, m_lineCnt);
    }
    // 更新绘图，这种方式再高填充下浪费资源
    //ui->customPlot->replot();
    // 此方法绘图避免重复绘图
    ui->customPlot->replot(QCustomPlot::rpQueuedReplot);
}

void MainWindow::slot_statusBarMessage(QString msg)
{
    ui->labComStatus->setText(msg);
}

void MainWindow::on_cbxPlan_stateChanged(int arg1)
{
    if(arg1)
    {
        m_sendData->start();
    }
    else
    {
        m_sendData->stop();
    }
}

//使能按键控制
void MainWindow::on_cbxIOEnable_stateChanged(int arg1)
{
    QByteArray sendData;
    quint16 sendLen;

    sendLen = 1;
    sendData.resize(sendLen);
    if(arg1)
        sendData[0] = 1;
    else
        sendData[0] = 0;
    m_deviceManager->sendData(m_devId, CMD_MC_SET_KEY_ENABLE, sendData, sendLen);
}

void MainWindow::onShowPopup()
{
    qDebug() << "afadsfasdf";
    updateDeviceList();
}

