#include"pid.h"

void Pid_control::PID_init(double kP,double Ki,double Kd,double expectedCTE,double ActualCTE,double deadZone)
{
	pid.SETCTE = expectedCTE;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.voltage = 0.0;
	pid.integral = 0.0;
	pid.Kp = kP;
	pid.Ki = Ki;
	pid.Kd = Kd;
	pid.ActualCTE = ActualCTE;
	pid.deadZone = deadZone;
	/*pid.Ki = 0.015;
	pid.Kd = 0.2;*/
}

double Pid_control::PID_realize(double CTE){
	pid.ActualCTE = CTE;
	pid.err = pid.SETCTE - pid.ActualCTE;
	
	//设置死区，防止抖动
	if (fabs(pid.err)<= pid.deadZone) {
		pid.voltage = 0;
	}
	else {
		pid.integral += pid.err;
		pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
		pid.err_last = pid.err;
	}
	return pid.voltage;
}

double Pid_control::getCTE() {//假设是线性的模型，测试用
	pid.ActualCTE += pid.voltage;
	return pid.ActualCTE;
}
