/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 现在升级为5路红外传感器，放置在小车前方
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/
#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H


#include "main.h"
#include <math.h>

//导入外部红外传感器数据变量
extern u8 Sensor_Left, Sensor_MiddleLeft, Sensor_Middle, Sensor_MiddleRight, Sensor_Right;
extern float Target_Velocity;

//导入外部红外传感器的PID参数
extern float Sensor_Kp, Sensor_KI, Sensor_Kd;

//读取红外传感器的值函数的声明
void Get_Sensor_Value(void);

//红外传感器PID控制函数的声明，返回类型统一为int
int Sensor_PID(void);

//初始化引脚
void Init_Sensor_Pins(void);

void Init_Sensor_Pins(void);

//以下五个宏定义分别对应五个红外传感器的引脚
//依次为最左、中左、中间、中右、最右
#define SENSOR1_PIN GPIO_PIN_0  // 左传感器
#define SENSOR1_PORT GPIOA

#define SENSOR2_PIN GPIO_PIN_13  // 中左传感器
#define SENSOR2_PORT GPIOC

#define SENSOR3_PIN GPIO_PIN_5   // 中间传感器(新增) - 请替换为实际可用的引脚
#define SENSOR3_PORT GPIOB

#define SENSOR4_PIN GPIO_PIN_14  // 中右传感器
#define SENSOR4_PORT GPIOC

#define SENSOR5_PIN GPIO_PIN_15  // 右传感器
#define SENSOR5_PORT GPIOC

// 参数和限制定义
#define INTEGRAL_LIMIT 3500    // 积分限幅
#define FILTER_SAMPLES 0      // 滤波采样次数

// 导入外部变量
extern volatile int Encoder_Left, Encoder_Right;
extern volatile int Balance_Pwm, Velocity_Pwm, Turn_Pwm;


#endif
