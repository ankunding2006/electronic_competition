/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 当检测到pid控制的error值过大时会降低小车的速度,以便更好的调整方向
# 红外传感器为4路,都放置在小车前方
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/

#include "main.h"
#include "LineFinder.h"

//小车速度限制量(对Encoder_Integral进行限制以便减小速度更好的调整方向)
u16 Speed_Limit = 1000;


void Get_Sensor_Value(void)
{
    //读取红外传感器的值
    Sensor_Left = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
    Sensor_MiddleLeft = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
    Sensor_MiddleRight = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    Sensor_Right = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
}

/*
介绍:PID控制函数,用于红外传感器的PID控制,并且在错误较大时候降低小车速度
返回值:u16类型的PWM值,从而在control.c调用,从而把返回值赋给Turn_Pwm(右转为正，左转为负)
函数参数:无
*/
