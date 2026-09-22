#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
#include "Encoder.h"
#include "Motor.h"
#include "Key.h"
#include "MPU6050.h"
#include "OLED.h"
#include "LED.h"
#include "math.h"
#include <string.h>
#include <stdlib.h>

#include "PID.h"
#include "BlueSerial.h"
#include "Serial.h"

uint8_t Ket_Status , Runflag, TimeOutFlag;
int16_t TimerCount;
int16_t AX,AY,AZ,GX, GY, GZ;
int16_t AvePWM,DifPWM,LeftPWM,RightPWM;

float LeftSpeed, RightSpeed;
float AveSpeed, DifSpeed;

float Angle_Acc;
float Angle_Gyro;
float Angle;


PID_t AnglePID = {
	.Kp = 5,
	.Ki = 0.1,
	.Kd = 5,
	
	.OutMax = 100,
	.OutMin = -100,
	.ErrorIntMax = 150,				//误差积分的最大值
	.ErrorIntMin = -150,
	.offset = 3.5,
};

PID_t SpeedPID = {
	.Kp = 2.5,
	.Ki = 0.07,
	.Kd = 0.05,
	.ErrorIntMax = 150,
	.ErrorIntMin = -150,
	.OutMax = 20,
	.OutMin = -20,
};

PID_t TurnPID = {
	.Kp = 4,
	.Ki = 3,
	.Kd = 0,
	
	.OutMax = 50,
	.OutMin = -50,
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};

int main(void){
	Encoder_Init();
	Motor_Init();
	Timer_Init();
	Key_Init();
	OLED_Init();
	LED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	Serial_Init();
	while(1){
		Ket_Status = Key_GetNum();
		if(Ket_Status==1){
			PID_Init(&AnglePID);
			PID_Init(&SpeedPID);
			PID_Init(&TurnPID);
			Runflag = 1;
			LED_ON();
		}
		else if(Ket_Status == 2 || Runflag == 0){
			Runflag = 0;
			LED_OFF();
		}
		OLED_Clear();
		//X:0-88列；Y：0-48行；每行相差8
		//角度环
		OLED_Printf(0, 0, OLED_6X8, "  Angle");						
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);		
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);		
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);	
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);			
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.1f", AnglePID.Out);

		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);				
		OLED_Printf(56, 56, OLED_6X8, "offset:%02.1f", AnglePID.offset);

		//速度环
		OLED_Printf(50, 0, OLED_6X8, "Speed");	
		OLED_Printf(50, 8, OLED_6X8, "%05.2f", SpeedPID.Kp);		
		OLED_Printf(50, 16, OLED_6X8, "%05.2f", SpeedPID.Ki);		
		OLED_Printf(50, 24, OLED_6X8, "%05.2f", SpeedPID.Kd);		
		OLED_Printf(50, 32, OLED_6X8, "%+05.1f", SpeedPID.Target);	
		OLED_Printf(50, 40, OLED_6X8, "%+05.1f", AveSpeed);			
		OLED_Printf(50, 48, OLED_6X8, "%+05.1f", SpeedPID.Out);	
		
		//转向环
		OLED_Printf(88, 0, OLED_6X8, "Turn");	
		OLED_Printf(88, 8, OLED_6X8, "%05.2f", TurnPID.Kp);			
		OLED_Printf(88, 16, OLED_6X8, "%05.2f", TurnPID.Ki);		
		OLED_Printf(88, 24, OLED_6X8, "%05.2f", TurnPID.Kd);		
		OLED_Printf(88, 32, OLED_6X8, "%+05.1f", TurnPID.Target);	
		OLED_Printf(88, 40, OLED_6X8, "%+05.1f", DifSpeed);			
		OLED_Printf(88, 48, OLED_6X8, "%+05.1f", TurnPID.Out);	

		OLED_Update();
		
		
		/*蓝牙串口接收数据包处理*/
		/*规定的数据包格式为：[数据1,数据2,数据3,...]*/
		if (BlueSerial_RxFlag == 1)		//如果收到数据包
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");	//提取数据1，定义为标签Tag
			if (strcmp(Tag, "key") == 0)					//Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为按键名称
				char *Action = strtok(NULL, ",");			//提取数据3，定义为按键动作
				
				/*此处可执行按键操作，目前程序暂时没用到按键*/
			}
			else if (strcmp(Tag, "slider") == 0)			//Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ",");			//提取数据3，定义为滑杆值
				
				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0)			//如果滑杆名称是AngleKp
				{
					AnglePID.Kp = atof(Value);				//则把滑杆值赋值给角度环Kp
				}
				else if (strcmp(Name, "AngleKi") == 0)		//如果滑杆名称是AngleKi
				{
					AnglePID.Ki = atof(Value);				//则把滑杆值赋值给角度环Ki
				}
				else if (strcmp(Name, "AngleKd") == 0)		//如果滑杆名称是AngleKd
				{
					AnglePID.Kd = atof(Value);				//则把滑杆值赋值给角度环Kd
				}
				else if (strcmp(Name, "SpeedKp") == 0)		//如果滑杆名称是SpeedKp
				{
					SpeedPID.Kp = atof(Value);				//则把滑杆值赋值给速度环Kp
				}
				else if (strcmp(Name, "SpeedKi") == 0)		//如果滑杆名称是SpeedKi
				{
					SpeedPID.Ki = atof(Value);				//则把滑杆值赋值给速度环Ki
				}
				else if (strcmp(Name, "SpeedKd") == 0)		//如果滑杆名称是SpeedKd
				{
					SpeedPID.Kd = atof(Value);				//则把滑杆值赋值给速度环Kd
				}
			}
			else if (strcmp(Tag, "joystick") == 0)			//Tag为joystick，收到摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ","));		//提取数据2，定义为摇杆值LH
				int8_t LV = atoi(strtok(NULL, ","));		//提取数据3，定义为摇杆值LV
				int8_t RH = atoi(strtok(NULL, ","));		//提取数据4，定义为摇杆值RH
				int8_t RV = atoi(strtok(NULL, ","));		//提取数据5，定义为摇杆值RV
				
				/*执行摇杆操作*/
				SpeedPID.Target = LV / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
				TurnPID.Target = RH / 25.0;				//摇杆值RH缩放后，控制差分PWM，左右转弯控制
			}
			
			BlueSerial_RxFlag = 0;				//处理完成后，标志位置0，允许接收下一个数据包
		}
		
		/*蓝牙串口打印波形，需配合蓝牙串口小程序实现波形绘制*/
		BlueSerial_Printf("[plot,%f,%f]", SpeedPID.Target, AveSpeed);	//绘制SpeedPID.Target和AveSpeed的波形
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count_Angle, Count_Speed;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{   
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		Count_Angle ++;
		Count_Speed++;
		Key_Tick();
		if(Count_Angle == 10){
			Count_Angle = 0;
			MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
			GY += 24;
			AX+=30;
			//利用重力加速度计来获取倾斜角
			Angle_Acc = -atan2(AX, AZ) / 3.14159 * 180;
			Angle_Acc += 0.3;
			/*公式中32768是int16_t变量的最大值，2000是陀螺仪配置的满量程2000度每秒，0.01是间隔时间10ms*/
			Angle_Gyro = Angle+GY/32768.0 * 2000 * 0.01;
			
			float Alpha = 0.01;
			Angle = Alpha*Angle_Acc + (1-Alpha)*Angle_Gyro;
			if(Angle > 45 || Angle < -45){
				Runflag = 0;
			}
			
			if(Runflag){
				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;
				
				LeftPWM = AvePWM+DifPWM/2;
				RightPWM = AvePWM - DifPWM/2;
				
				/*PWM限幅*/
				/*上式计算后，LeftPWM和RightPWM可能会超出电机允许的PWM范围，此处将PWM值范围限制在-100~100之内*/
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				/*PWM输出给电机*/
				Motor_SetPWM(1, LeftPWM);		//LeftPWM输出给左轮电机
				Motor_SetPWM(2, RightPWM);		//RightPWM输出给右轮电机
				
			}
			else{
				Motor_SetPWM(1,0);
				Motor_SetPWM(2,0);
			}
		}
		
		if(Count_Speed == 50){
			Count_Speed = 0;
			LeftSpeed = Encoder_Get(1)/ 44.0 / 0.05 / 9.27666;
			RightSpeed = Encoder_Get(2)/ 44.0 / 0.05 / 9.27666;
			DifSpeed = LeftSpeed - RightSpeed;
			AveSpeed = (LeftSpeed + RightSpeed)/2;
			
			if(Runflag){
				SpeedPID.Actual = AveSpeed;//速度环调控
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;
				
				TurnPID.Actual = DifSpeed;//角度环调控
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
			}
		}
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/*标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
			/*置TimerErrorFlag为1，表示定时中断错误*/
			TimeOutFlag = 1;
			
			/*清标志位，避免中断连续触发，导致主函数完全无法执行*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		
		/*中断函数退出前，读取计数器的值，此值可用于测量中断函数的具体执行时间*/
		TimerCount = TIM_GetCounter(TIM1);
	}
}

