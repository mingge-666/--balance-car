#include "stm32f10x.h"                  // Device header
#include "PID.h"

void PID_Init(PID_t *p){
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	
	p->Target = 0;
	
	p->Actual = 0;
	p->offset = 5;
	
	p->Out = 0;
}

void PID_Update(PID_t *p){
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	if (p->Ki != 0)				//如果Ki不为0，就积分，为0就先不积分
	{
		p->ErrorInt += (p->Error0)/(0.3*p->Error0+0.9);	//进行误差积分,变速积分
	}
	else							
	{
		p->ErrorInt = 0;			//误差积分直接归0
	}
	if(p->ErrorInt>300){p->ErrorInt = 300;}//积分限幅，防止意外爆冲
	if(p->ErrorInt<-300){p->ErrorInt = -300;}
	p->Pre_ErrorDer = p->ErrorDer;
	p->ErrorDer = p->Error0 - p->Error1;
	p->Out = p->Kp*p->Error0 + p->Ki * p->ErrorInt + 
	p->Kd * (0.6*p->ErrorDer+0.4*p->Pre_ErrorDer);//不完全微分来减弱噪音
	
	if(p->Out<5 && p->Out > 0){p->Out = p->offset;}//积分偏移
	if(p->Out>-5 && p->Out < 0){p->Out = -p->offset;}
	
	if(p->Out > p->OutMax){p->Out = p->OutMax;}
	if(p->Out < p->OutMin){p->Out = p->OutMin;}
}

