#ifndef COMMUNICATIONFACTORY_H
#define COMMUNICATIONFACTORY_H

#include "AbstractCommunication.h"
#include "TinyCarCommunication.h"
#include "LargeCarWindowsCommunication.h"
#include "LargeCarLinuxCommunication.h"
#include <memory>
enum CommunicationType{LargeCarWindows,TinyCarWindows,LargeCarLinux,TinyCarLinux};

class CommunicationFactory{
public:
    static AbstractCommunication* createCommunication(CommunicationType type){
        switch (type){
        case LargeCarWindows:
            qDebug()<<"LargeCarWindowsCommunication open";
            //return new LargeCarWindowsCommunication();break;
        case TinyCarLinux:
            qDebug()<<"TinyCarCommunication open";
            return new TinyCarCommunication();
        case TinyCarWindows:
            qDebug()<<"TinyCarCommunication open";
            return new TinyCarCommunication();
        case LargeCarLinux:
            qDebug()<<"LargeCarLinuxCommunication open";
            return new LargeCarLinuxCommunication();
        default: return NULL;
        }
    }
};
#endif // COMMUNICATIONFACTORY_H
