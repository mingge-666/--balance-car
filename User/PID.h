#ifndef __PID_H
#define __PID_H

typedef struct{
	float Target;
	float Actual;
	float Actual1;
	
	float Error0; 
	float Error1;
	float ErrorInt;
	float ErrorIntMax;
	float ErrorIntMin;
	float ErrorDer;
	float Pre_ErrorDer;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Out;
	float OutMax;
	float OutMin;
	
	float offset;
}PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);


#endif
