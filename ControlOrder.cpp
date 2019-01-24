#include "ControlOrder.h"

#include<QString>
void ControlOrder::init(){
    this->speed=0;
    this->turnRange=0;
    this->gear=0;
    this->lightSignal=0;
    this->horn=0;
}

ControlOrder& ControlOrder::setSpeed(qint16 speed){
//    if(speed>CONTROLORDER_MAX_SPEED){
//        speed=CONTROLORDER_MAX_SPEED;
//    }else if(speed<CONTROLORDER_MIN_SPEED){
//        speed=CONTROLORDER_MIN_SPEED;
//    }
    this->speed=speed;
    return *this;
}

ControlOrder& ControlOrder::setTurnRange(qint16 turnRange){
//    if(turnRange>CONTROLORDER_MAX_TURN_RANGE){
//        turnRange=CONTROLORDER_MAX_TURN_RANGE;
//    }else if(turnRange<CONTROLORDER_MIN_TURN_RANGE){
//        turnRange=CONTROLORDER_MIN_TURN_RANGE;
//    }
    this->turnRange=turnRange;
    return *this;
}

ControlOrder& ControlOrder::setGear(qint8 gear){
    if(gear==CONTROLORDER_GEAR_ZERO||gear==CONTROLORDER_GEAR_FORWARD||gear==CONTROLORDER_GEAR_BACKWARD){
        this->gear=gear;
    }else{
        qDebug("geal=%d,setting error（挡位设置异常",gear);
    }
    return *this;
}

ControlOrder& ControlOrder::setLightSignal(qint8 lightSignal){
    if(lightSignal==CONTROLORDER_LIGHT_ALL_OFF||lightSignal==CONTROLORDER_LIGHT_LEFT_ON||lightSignal==CONTROLORDER_LIGHT_RIGHT_ON){
        this->lightSignal=lightSignal;
    }else{
        qDebug("lightSignal=%d,setting error（转向灯设置异常",lightSignal);
    }
    return *this;
}

ControlOrder& ControlOrder::setHorn(qint8 horn){
    if(horn==CONTROLORDER_HORN_OFF||horn==CONTROLORDER_HORN_ON){
        this->horn=horn;
    }else{
        qDebug("horn=%d,setting error（鸣笛设置异常",horn);
    }
    return *this;
}

void ControlOrder::printInfo(){
    qDebug("Control Order: speed:%d,turnRange:%d,gear:%d%s,lightSignal:%d%s,hore:%d%s",
           speed,turnRange,
           gear,gear==0?("空挡"):gear==1?("前进挡"):("后退档"),
           lightSignal,lightSignal==0?("所有灯光关闭"):lightSignal==1?("右转向灯亮"):("左转向灯亮"),
           horn,horn==0?("静音"):("鸣笛")
           );
}
