#-------------------------------------------------
#
# Project created by QtCreator 2019-01-04T15:23:13
#
#-------------------------------------------------

QT       += core gui
QT       += serialport
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtControl
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        MainWindow.cpp \
    SerialPortDialog.cpp \
    SerialInfoDialog.cpp \
    PaintWidget.cpp \
    CoTrans.cpp \
    nmeaparser.cpp \
    nmeaPres.cpp \
    nmeautils.cpp \
    ControlOrder.cpp \
    TinyCarCO.cpp \
    LargeCarCO.cpp \
    GpsBufferWriteThread.cpp \
    GpsBufferReadThread.cpp \
    GpsBufferReadInitRouteThread.cpp \
    RouteSparseDialog.cpp \
    InitRouteDialog.cpp \
    GpsBufferConsumeRunThread.cpp \
    ProcessRunDialog.cpp \
    PID.cpp \
    GPSProcesser.cpp \
    ControlOrderSendThread.cpp \
    LargeCarWindowsCommunication.cpp \
    TinyCarCommunication.cpp \
    TinyCarSerialPortDialog.cpp \
    SocketSettingWidget.cpp \
    Processer.cpp \
    ProcessRunNoGPSDialog.cpp \
    LocationBufferConsumeRunThread.cpp \
    LocationBufferProduceThread.cpp \
    LocationBufferConsumInitRouteThread.cpp \
    LocationInitRouteDialog.cpp \
    RosSettingDialog.cpp \
    RosReceiveThread.cpp \
    LargeCarLinuxCommunication.cpp \
    ultrasonic.cpp \
    fileoperation.cpp \
    RosPerceptionReceiveThread.cpp

HEADERS += \
        MainWindow.h \
    SerialPortDialog.h \
    RingBuffer.h \
    SerialInfoDialog.h \
    PaintWidget.h \
    CoTrans.h \
    nmeaparser.h \
    nmeaPres.h \
    nmeautils.h \
    ControlOrder.h \
    TinyCarCO.h \
    LargeCarCO.h \
    ControlMessageStruct.h \
    GpsInfo.h \
    GpsBufferWriteThread.h \
    GpsBufferReadInitRouteThread.h \
    RouteSparseDialog.h \
    InitRouteDialog.h \
    GpsBufferReadThread.h \
    GpsBufferConsumeRunThread.h \
    ProcessRunDialog.h \
    PID.h \
    PID.h \
    GPSProcesser.h \
    ControlOrderSendThread.h \
    AbstractCommunication.h \
    CommunicationFactory.h \
    LargeCarWindowsCommunication.h \
    TinyCarCommunication.h \
    TinyCarSerialPortDialog.h \
    PreDefinition.h \
    SocketSettingWidget.h \
    Processer.h \
    ProcessRunNoGPSDialog.h \
    LocationPosition.h \
    LocationBufferConsumeRunThread.h \
    LocationBufferProduceThread.h \
    LocationBufferConsumInitRouteThread.h \
    LocationInitRouteDialog.h \
    RosSettingDialog.h \
    RosReceiveThread.h \
    ControlCAN_Win.h \
    ControlCAN.h \
    LargeCarLinuxCommunication.h \
    ultrasonic.h \
    fileoperation.h \
    RosPerceptionReceiveThread.h \
    distance.h \
    distances.h

FORMS += \
        MainWindow.ui \
    SerialInfoDialog.ui \
    RouteSparseDialog.ui \
    InitRouteDialog.ui \
    ProcessRunDialog.ui \
    TinyCarSerialPortDialog.ui \
    SocketSettingWidget.ui \
    locationinitroutedialog.ui \
    RosSettingDialog.ui

RESOURCES += \
    res.qrc

win32:{
    INCLUDEPATH+= D:\local\boost_1_67_0
    LIBS += -L$$PWD/./ -lControlCAN
    LIBS += -L$$PWD/./ -llibboost_atomic-vc141-mt-gd-x64-1_67
}

linux-g++*{

    ##--add boost
    #INCLUDEPATH += /home/zhb/Downloads/boost_1_67_0

    #--add ros libs

#    LIBS += -L /opt/ros/melodic/lib/ -lroscpp
#    LIBS += -L /opt/ros/melodic/lib/ -lroslib
#    LIBS += -L /opt/ros/melodic/lib/ -lpthread
#    LIBS += -L /opt/ros/melodic/lib/ -lroscpp_serialization
#    LIBS += -L /opt/ros/melodic/lib/ -lrostime
#    LIBS += -L /opt/ros/melodic/lib/ -lrosconsole
#    LIBS += -L /opt/ros/melodic/lib/ -lrosconsole_log4cxx
#    LIBS += -L /opt/ros/melodic/lib/ -lrosconsole_backend_interface
#    LIBS += -L /opt/ros/melodic/lib/ -lxmlrpcpp

#    LIBS += -L$$PWD/../../../../opt/ros/melodic/lib/ -lcpp_common

#    INCLUDEPATH += $$PWD/../../../../opt/ros/melodic/include
#    DEPENDPATH += $$PWD/../../../../opt/ros/melodic/include

#    LIBS += -L$$PWD/../build-QtControl-Desktop_Qt_5_12_2_GCC_64bit-Debug/ -lcontrolcan

    #IPC config start
    #--add ros include
    INCLUDEPATH += -I /opt/ros/kinetic/include
    DEPENDPATH +=  /opt/ros/kinetic/include

    #--add ros libs

    LIBS += -L /opt/ros/kinetic/lib/ -lroscpp
    LIBS += -L /opt/ros/kinetic/lib/ -lroslib
    LIBS += -L /opt/ros/kinetic/lib/ -lpthread
    LIBS += -L /opt/ros/kinetic/lib/ -lroscpp_serialization
    LIBS += -L /opt/ros/kinetic/lib/ -lrostime
    LIBS += -L /opt/ros/kinetic/lib/ -lrosconsole
    LIBS += -L /opt/ros/kinetic/lib/ -lrosconsole_log4cxx
    LIBS += -L /opt/ros/kinetic/lib/ -lrosconsole_backend_interface
    LIBS += -L /opt/ros/kinetic/lib/ -lxmlrpcpp

    LIBS += -L$$PWD/../../../../opt/ros/kinetic/lib/ -lcpp_common

    INCLUDEPATH += $$PWD/../../../../opt/ros/kinetic/include
    DEPENDPATH += $$PWD/../../../../opt/ros/kinetic/include

    #CAN drivers

    LIBS += -L$$PWD/../build-QtControl-Desktop_Qt_5_9_0_GCC_64bit-Debug -lcontrolcan
    #IPC config end

}

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

#win32: LIBS += -L$$PWD/./ -llibboost_chrono-vc141-mt-gd-x64-1_67

