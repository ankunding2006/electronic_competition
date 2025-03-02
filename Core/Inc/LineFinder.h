/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 红外传感器为4路,都放置在小车前方
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/
#include "main.h"
#include <math.h>
//导入外部红外传感器数据变量
extern u8 Sensor_Left,Sensor_MiddleLeft,Sensor_MiddleRight,Sensor_Right;
extern float Target_Velocity;

//导入外部红外传感器的PID参数
extern float Sensor_Kp,Sensor_KI,Sensor_Kd;
//读取红外传感器的值函数的声明
void Get_Sensor_Value(void);
//红外传感器PID控制函数的声明，返回类型统一为int
int Sensor_PID(void);
//以下四个宏定义分别对应四个红外传感器的引脚
//依次为左、中左、中右、右
#define SENSOR1_PIN GPIO_PIN_0
#define SENSOR1_PORT GPIOA

#define SENSOR2_PIN GPIO_PIN_13
#define SENSOR2_PORT GPIOC

#define SENSOR3_PIN GPIO_PIN_14
#define SENSOR3_PORT GPIOC

#define SENSOR4_PIN GPIO_PIN_15
#define SENSOR4_PORT GPIOC

//积分限幅宏定义
#define INTEGRAL_LIMIT 30

#define Error_threshold 2

extern volatile int Encoder_Left,Encoder_Right;             					//左右编码器的脉冲计数
extern volatile int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float base_velocity;
