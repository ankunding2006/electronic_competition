/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 当检测到pid控制的error值过大时会降低小车的速度,以便更好的调整方向
# 红外传感器为4路,都放置在小车前方
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/

#include "main.h"
#include "LineFinder.h"

float base_velocity = 0;
u16 Speed_Limit = 1000;

// 滤波相关参数
#define FILTER_SAMPLES 5  // 采样次数
u8 sensor_history[4][FILTER_SAMPLES]; // 传感器历史数据
u8 filter_index = 0;      // 当前采样索引

/*
介绍: 获取传感器数据并进行滤波处理
功能: 读取四路红外传感器，并使用均值滤波处理
参数: 无
返回: 无
*/
void Get_Sensor_Value(void)
{
    u8 i;
    u8 sum_left = 0, sum_middle_left = 0, sum_middle_right = 0, sum_right = 0;
    
    // 读取当前传感器值
    sensor_history[0][filter_index] = HAL_GPIO_ReadPin(SENSOR1_PORT, SENSOR1_PIN);
    sensor_history[1][filter_index] = HAL_GPIO_ReadPin(SENSOR2_PORT, SENSOR2_PIN);
    sensor_history[2][filter_index] = HAL_GPIO_ReadPin(SENSOR3_PORT, SENSOR3_PIN);
    sensor_history[3][filter_index] = HAL_GPIO_ReadPin(SENSOR4_PORT, SENSOR4_PIN);
    
    // 更新索引
    filter_index = (filter_index + 1) % FILTER_SAMPLES;
    
    // 计算平均值
    for(i = 0; i < FILTER_SAMPLES; i++) {
        sum_left += sensor_history[0][i];
        sum_middle_left += sensor_history[1][i];
        sum_middle_right += sensor_history[2][i];
        sum_right += sensor_history[3][i];
    }
    
    // 基于阈值确定最终传感器状态
    // 如果超过一半的采样是高电平，则认为检测到黑线
    Sensor_Left = (sum_left > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_MiddleLeft = (sum_middle_left > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_MiddleRight = (sum_middle_right > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_Right = (sum_right > (FILTER_SAMPLES / 2)) ? 1 : 0;
}

/*
介绍: PID控制函数,用于红外传感器的PID控制,并且在错误较大时候降低小车速度
返回值: int类型的PWM值,从而在control.c调用,从而把返回值赋给Turn_Pwm(右转为正，左转为负)
函数参数: 无
*/
int Sensor_PID(void)
{
    static float error = 0, last_error = 0;
    static float P = 0, I = 0, D = 0;
    float PID_value = 0;
    static u8 speed_reduced = 0;  // 标记速度是否已被降低
    
    // 读取传感器状态并滤波
    Get_Sensor_Value();
    
    // 通过传感器状态计算偏差
    // 不同传感器组合对应不同的偏差值
    if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_MiddleRight == 0 && Sensor_Right == 0)
        error = 0; // 全白，可能偏离轨道，保持当前方向
    else if (Sensor_Left == 1 && Sensor_MiddleLeft == 0 && Sensor_MiddleRight == 0 && Sensor_Right == 0)
        error = -3; // 强左转
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 1 && Sensor_MiddleRight == 0 && Sensor_Right == 0)
        error = -1.5; // 左转
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_MiddleRight == 1 && Sensor_Right == 0)
        error = 1.5; // 右转
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_MiddleRight == 0 && Sensor_Right == 1)
        error = 3; // 强右转
    else if (Sensor_Left == 1 && Sensor_MiddleLeft == 1 && Sensor_MiddleRight == 0 && Sensor_Right == 0)
        error = -2; // 偏左
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_MiddleRight == 1 && Sensor_Right == 1)
        error = 2; // 偏右
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 1 && Sensor_MiddleRight == 1 && Sensor_Right == 0)
        error = 0; // 居中，直行
    else if (Sensor_Left == 1 && Sensor_MiddleLeft == 1 && Sensor_MiddleRight == 1 && Sensor_Right == 1)
        error = 0; // 全黑，可能是交叉路口，直行
    
    // 计算PID值
    P = error;
    I = I + error; // 积分项累加当前误差
    D = error - last_error; // 微分项为当前误差减去上次误差
    
    // I值限幅，防止积分饱和
    if (I > INTEGRAL_LIMIT) I = INTEGRAL_LIMIT;
    if (I < -INTEGRAL_LIMIT) I = -INTEGRAL_LIMIT;
    
    // 使用PID所有三个分量计算输出值
    // 将PID参数缩小100倍，因为在定义时已经放大了100倍
    PID_value = (Sensor_Kp/100.0) * P + (Sensor_KI/100.0) * I + (Sensor_Kd/100.0) * D;
    
    // 保存上一次误差
    last_error = error;
    
    // 根据误差大小动态调整速度
    // 当误差超过阈值时降低速度，误差恢复时恢复速度
    // 根据误差大小动态调整速度
    // 当误差超过阈值时降低速度，误差恢复时恢复速度
    if (fabs(error) >= Error_threshold) {
        if (!speed_reduced) {
            // 误差刚超过阈值，首次降低速度，先保存原始速度
            base_velocity = Target_Velocity;  // 无论如何都保存当前速度
            Target_Velocity = base_velocity * 0.7; // 降低速度到基准速度的70%
            speed_reduced = 1; // 标记速度已降低
        }
    } else if (fabs(error) < Error_threshold && speed_reduced) {
        // 误差恢复到较小值，恢复原来的速度
        Target_Velocity = base_velocity;
        speed_reduced = 0; // 清除降速标记
    }
    
    // 限制PID输出范围，防止转向过猛
    if (PID_value > Speed_Limit) PID_value = Speed_Limit;
    if (PID_value < -Speed_Limit) PID_value = -Speed_Limit;
    
    // 返回PID值，保持正负号以确保方向正确
    return (int)PID_value;
}
