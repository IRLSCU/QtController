#include "TinyCarCO.h"

TinyCarCO& TinyCarCO::setLeftSpeed(quint8 leftSpeed){
//    if(leftSpeed<TINYCARCO_MIN_SPEED){
//        leftSpeed=TINYCARCO_MIN_SPEED;
//    }else if(leftSpeed>TINYCARCO_MAX_SPEED){
//        leftSpeed=TINYCARCO_MAX_SPEED;
//    }
    this->leftSpeed=leftSpeed;
    return *this;
}
TinyCarCO& TinyCarCO::setRightSpeed(quint8 rightSpeed){
//    if(rightSpeed<TINYCARCO_MIN_SPEED){
//        rightSpeed=TINYCARCO_MIN_SPEED;
//    }else if(rightSpeed>TINYCARCO_MAX_SPEED){
//        rightSpeed=TINYCARCO_MAX_SPEED;
//    }
    this->rightSpeed=rightSpeed;
    return *this;
}
TinyCarCO& TinyCarCO::setLeftOrientation(qint8 leftOrientation){
    if(leftOrientation==TINYCARCO_ORIENTATION_BACKWARD||leftOrientation==TINYCARCO_ORIENTATION_FORWARD){
        this->leftOrientation=leftOrientation;
    }
    return *this;
}

TinyCarCO& TinyCarCO::setRightOrientation(qint8 rightOrientation){
    if(rightOrientation==TINYCARCO_ORIENTATION_BACKWARD||leftOrientation==TINYCARCO_ORIENTATION_FORWARD){
        this->rightOrientation=rightOrientation;
    }
    return *this;
}

void TinyCarCO::init(){
    this->leftSpeed=0;
    this->rightSpeed=0;
    this->leftOrientation=0;
    this->rightOrientation=0;
}
