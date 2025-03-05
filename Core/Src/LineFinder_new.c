#include "sensor.h"
#include "stm32f10x.h"
#include "move.h"
#include "motor.h"
#include "FSM.h"
 
//STEER4 		--> PA11  --> R2  红线
//STEER3 		--> PC9 	--> R1  橘线
//			 		--> PB4   --> M0  黄线
//STEER1 		--> PA6   --> L1  绿线
//ENCODE1_A --> PB5	  --> L2  棕线
 
float Kp_sensor = 8.134, Ki_sensor = 0.021, Kd_sensor = 2.36;//pid弯道参数参数 
float sensor_bias = 0;
float sensor_bias_last = 0;
float P = 0, I = 0, D = 0, PID_value = 0;  //pid直道参数 
int decide;
 
unsigned char move_flag;
extern unsigned char FSM_state;
void sensor_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//开启C时钟   PC9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//这句话其实可以不用，在使用输入功能时，不需要配置频率
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启A时钟  PA11  PA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开启B时钟  PB4  PB5
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
 
unsigned char times;
extern unsigned char FSM_hc08;
unsigned char back_flag;
unsigned char one_time;
unsigned char one_flag;
unsigned char channel_num;//channel的存在导致发送2号通道的时候只会进入一次
void sensor_read()
{
	if((L2 == 1)&&(L1 == 1)&&(M0 == 1)&&(R1 == 0))// 1 1 1 0 
	{
		if(one_time == 0)
		{
			if(FSM_hc08 == Channel_1)
			{
				FSM_state = Turn_lift_state;//跳出循环，将PID分别清零。
				one_time++;
				P = 0;I = 0; D = 0;
			}
			else if(FSM_hc08 == Channel_2)
			{
				if(one_flag == 0)
				{
					channel_num++;
					one_flag = 1;
				}
				if(channel_num == 2)
				{	
					FSM_state = stay2_state;//跳出循环，将PID分别清零。
					one_time++;
					P = 0;I = 0; D = 0;
				}
			}
		}
	}
	else if((L1 == 0)&&(M0 == 1)&&(R1 == 0))// 0 1 0 
	{
		sensor_bias = 0;decide = 3;one_flag = 0;//积分项清零
	}
	else if((L1 == 1)&&(M0 == 1)&&(R1 == 0))// 1 1 0 
	{
		sensor_bias = -62.5;decide = 2;
	}
	else if((L1 == 0)&&(M0 == 1)&&(R1 == 1))// 0 1 1 
	{
		sensor_bias = 62.5;decide = 2;
	}
	else if((L1 == 1)&&(M0 == 0)&&(R1 == 0))// 1 0 0 
	{
		sensor_bias = -125;decide = 4;
	}
	else if((L1 == 0)&&(M0 == 0)&&(R1 == 1))// 0 0 1 
	{
		sensor_bias = 62.5;decide = 4;
	}
	else if((L1 == 0)&&(M0 == 0)&&(R1 == 0))// 0 0 0 
	{
		decide = 6;
	}
	else if((L1 == 1)&&(M0 == 1)&&(R1 == 1))// 1 1 1 
	{
		decide = 6;FSM_state = Judge_state;		//如果读取到了整条黑线，那么就进入下一状态
		if(back_flag == 1)
		{
			FSM_state = Back_state;
			back_flag = 0;
		}			//第一次识别到全黑线为小车停止线。第二次识别到，代表小车即将回归循迹
	}
 
}
void Sensor_pid()
{
	if(decide<=5)
	{
		P = sensor_bias;
		I = I + sensor_bias;
		D = sensor_bias-sensor_bias_last;
		PID_value = Kp_sensor*P + Ki_sensor*I + Kd_sensor*D;
		sensor_bias_last = sensor_bias;
		//对积分值设置一个限制，防止积分值超标
		
		if(I >=3500)I = 3500;
		if(I <= -3500)I = -3500;
		PWM_value_R = 2099 - (int)PID_value;
		PWM_value_L = 2099 + (int)PID_value;//当线在左，左轮要慢，右轮要快，左轮要加，右轮要减，但这里的偏差是负值
		Motor3_forward(PWM_value_R);
		Motor4_forward(PWM_value_L);
		//4 --> 右电机
		//3 --> 左电机	
	}
	else{
		Move_stop();
	}
}
