#pragma once

#ifndef PID_H
#define PID_H
#include<math.h>

typedef struct _pid {
	double SETCTE;//定义设定值  相当于预定偏离程度。SETCTE=0;
	double err;//定义偏差值
	double err_last;//定义上一个偏差值
	double Kp, Ki, Kd;//定义比例、积分、微分系数
	double voltage;//定义turnRange（控制执行器的变量）
	double integral;//定义积分值
	double ActualCTE;
	double deadZone;//死区范围防抖动
}Pid;

class Pid_control {
public:
	void PID_init(double Kp, double Ki, double Kd, double expectedCTE, double ActualCTE, double deadZone=0);//初始化Kp,Ki,Kd,以及目标值,起始值偏差,死区
    double PID_realize(double CTE);//传入实测值，返回差值
	double getCTE();//方便测试，是实际场景下的建模
private:
	int index;
	Pid pid;
};
#endif // !PID_H
