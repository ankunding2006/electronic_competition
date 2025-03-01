/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 红外传感器为4路,都放置在小车前方
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/
#include "main.h"

//导入外部红外传感器数据变量
extern u8 Sensor_Left,Sensor_MiddleLeft,Sensor_MiddleRight,Sensor_Right;
extern float Target_Velocity;

//导入外部红外传感器的PID参数
extern float Sensor_Kp,Sensor_Kd;
//读取红外传感器的值函数的声明
void Get_Sensor_Value(void);
//红外传感器PID控制函数的声明
u16 Sensor_PID(void);