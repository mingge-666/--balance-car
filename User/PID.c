#include "stm32f10x.h"                  // Device header
#include "PID.h"

void PID_Init(PID_t *p){
	p->Error0 = 0;
	p->Error1 = 0;

	p->ErrorInt = 0;
	
	p->Target = 0;
	
	p->Actual = 0;
	p->Actual1 = 0;
	
	p->Out = 0;
}

void PID_Update(PID_t *p){
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	if (p->Ki != 0)				//如果Ki不为0，就积分，为0就先不积分
	{
		p->ErrorInt += p->Error0;	//进行误差积分,变速积分
		if(p->ErrorInt>p->ErrorIntMax){p->ErrorInt = p->ErrorIntMax;}//积分限幅，防止意外爆冲
		if(p->ErrorInt<p->ErrorIntMin){p->ErrorInt = p->ErrorIntMin;}
	}
	else							
	{
		p->ErrorInt = 0;			//误差积分直接归0
	}
	
	p->Out = p->Kp*p->Error0 + p->Ki * p->ErrorInt - p->Kd * (p->Actual-p->Actual1);
	
	if(p->Out > 0){p->Out += p->offset;}//积分偏移
	if(p->Out < 0){p->Out -= p->offset;}
	
	if(p->Out > p->OutMax){p->Out = p->OutMax;}
	if(p->Out < p->OutMin){p->Out = p->OutMin;}
	
	p->Actual1 = p->Actual;
}

