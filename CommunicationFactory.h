#ifndef COMMUNICATIONFACTORY_H
#define COMMUNICATIONFACTORY_H

#include "AbstractCommunication.h"
#include "TinyCarCommunication.h"
#include "LargeCarWindowsCommunication.h"
#include <memory>
enum CommunicationType{LargeCarWindows,TinyCarWindows,LargeCarLinux,TinyCarLinux};

class CommunicationFactory{
public:
    static AbstractCommunication* createCommunication(CommunicationType type){
        switch (type){
        case LargeCarWindows:
            qDebug()<<"LargeCarWindowsCommunication open";
            return new LargeCarWindowsCommunication();break;
        case TinyCarLinux:
            qDebug()<<"TinyCarCommunication open";
            return new TinyCarCommunication();break;
        case TinyCarWindows:
            qDebug()<<"TinyCarCommunication open";
            return new TinyCarCommunication();break;
        default: return NULL;
        }
    }
};
#endif // COMMUNICATIONFACTORY_H
