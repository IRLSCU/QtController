#ifndef COMMUNICATIONFACTORY_H
#define COMMUNICATIONFACTORY_H

#include "AbstractCommunication.h"
#include "LargeCarWindowsCommunication.h"
#include <memory>
enum CommunicationType{LargeCarWindows,TinyCarWindows,LargeCarLinux,TinyCarWinLinux};

class CommunicationFactory{
public:
    static AbstractCommunication* createCommunication(CommunicationType type){
        switch (type){
        case LargeCarWindows: return new LargeCarWindowsCommunication();
        default: return NULL;
        }
    }
};
#endif // COMMUNICATIONFACTORY_H
