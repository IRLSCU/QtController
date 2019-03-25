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
            return new LargeCarWindowsCommunication();break;
        case TinyCarLinux:
            return new TinyCarCommunication();break;
        case TinyCarWindows:
            return new TinyCarCommunication();break;
        default: return NULL;
        }
    }
};
#endif // COMMUNICATIONFACTORY_H
