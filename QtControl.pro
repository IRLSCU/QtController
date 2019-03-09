#-------------------------------------------------
#
# Project created by QtCreator 2019-01-04T15:23:13
#
#-------------------------------------------------

QT       += core gui
QT       += serialport
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
    GPSProcesser.cpp

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
    GPSProcesser.h

FORMS += \
        MainWindow.ui \
    SerialInfoDialog.ui \
    RouteSparseDialog.ui \
    InitRouteDialog.ui \
    ProcessRunDialog.ui

RESOURCES += \
    res.qrc
INCLUDEPATH+= D:\local\boost_1_67_0
