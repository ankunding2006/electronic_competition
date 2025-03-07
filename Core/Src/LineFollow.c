/*
# 这个文件主要是储存与小车的红外循迹功能(使用PID的方式)相关的函数
# 升级为5路红外传感器，放置在小车前方，以实现更精确的循迹控制
# 传感器的输出值为0或1(分别对应低电平或高电平),1代表检测到黑线,0代表检测到白线
*/

#include "main.h"
#include "LineFollow.h"

#if FILTER_SAMPLES > 0
u8 sensor_history[5][FILTER_SAMPLES]; // 传感器历史数据，用于滤波
u8 filter_index = 0;                  // 当前滤波采样索引
#endif

u8 one_time = 0;    // 特殊状态标记
u8 one_flag = 0;    // 临时状态标记
u8 channel_num = 0; // 通道计数
/*
介绍: 获取传感器数据并进行滤波处理
功能: 读取五路红外传感器，并根据配置决定是否使用均值滤波处理
参数: 无
返回: 无
*/
void Get_Sensor_Value(void)
{
#if FILTER_SAMPLES > 0
    // 启用滤波功能的代码
    u8 i;
    u8 sum_left = 0, sum_middle_left = 0, sum_middle = 0, sum_middle_right = 0, sum_right = 0;

    // 读取当前传感器值
    sensor_history[0][filter_index] = HAL_GPIO_ReadPin(SENSOR1_PORT, SENSOR1_PIN); // 左
    sensor_history[1][filter_index] = HAL_GPIO_ReadPin(SENSOR2_PORT, SENSOR2_PIN); // 中左
    sensor_history[2][filter_index] = HAL_GPIO_ReadPin(SENSOR3_PORT, SENSOR3_PIN); // 中
    sensor_history[3][filter_index] = HAL_GPIO_ReadPin(SENSOR4_PORT, SENSOR4_PIN); // 中右
    sensor_history[4][filter_index] = HAL_GPIO_ReadPin(SENSOR5_PORT, SENSOR5_PIN); // 右

    // 更新滤波索引
    filter_index = (filter_index + 1) % FILTER_SAMPLES;

    // 计算平均值
    for (i = 0; i < FILTER_SAMPLES; i++)
    {
        sum_left += sensor_history[0][i];
        sum_middle_left += sensor_history[1][i];
        sum_middle += sensor_history[2][i];
        sum_middle_right += sensor_history[3][i];
        sum_right += sensor_history[4][i];
    }

    // 基于阈值确定最终传感器状态
    // 如果超过一半的采样是高电平，则认为检测到黑线
    Sensor_Left = (sum_left > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_MiddleLeft = (sum_middle_left > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_Middle = (sum_middle > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_MiddleRight = (sum_middle_right > (FILTER_SAMPLES / 2)) ? 1 : 0;
    Sensor_Right = (sum_right > (FILTER_SAMPLES / 2)) ? 1 : 0;
#else
    // 不使用滤波，直接读取当前值
    Sensor_Left = HAL_GPIO_ReadPin(SENSOR1_PORT, SENSOR1_PIN);        // 左
    Sensor_MiddleLeft = HAL_GPIO_ReadPin(SENSOR2_PORT, SENSOR2_PIN);  // 中左
    Sensor_Middle = HAL_GPIO_ReadPin(SENSOR3_PORT, SENSOR3_PIN);      // 中
    Sensor_MiddleRight = HAL_GPIO_ReadPin(SENSOR4_PORT, SENSOR4_PIN); // 中右
    Sensor_Right = HAL_GPIO_ReadPin(SENSOR5_PORT, SENSOR5_PIN);       // 右
#endif
}
/*
介绍: 传感器的PID控制函数
功能: 根据传感器状态计算转向PWM值
参数: 无
返回: 转向PWM值 (右转为正，左转为负)
*/
int Sensor_PID(void)
{
    static float sensor_bias = 0;      // 当前偏差
    static float sensor_bias_last = 0; // 上一次偏差
    static float P = 0, I = 0, D = 0;  // PID分量
    float PID_value = 0;               // PID计算结果
    int decide = 0;                    // 状态判断

    // 读取传感器值
    Get_Sensor_Value();

    // 根据传感器组合判断偏差，采用新的循迹逻辑

    // 特殊情况：全黑线或特定组合，可能是十字路口或T形路口
    if (Sensor_Left == 1 && Sensor_MiddleLeft == 1 && Sensor_Middle == 1 &&
        Sensor_MiddleRight == 1 && Sensor_Right == 1)
    {
        // 全黑状态，可能是十字路口
        decide = 6;
        return 0; // 在十字路口直行
    }
    // 特殊情况：检测到特定模式
    else if (Sensor_Left == 1 && Sensor_MiddleLeft == 1 && Sensor_Middle == 1 &&
             Sensor_MiddleRight == 0 && Sensor_Right == 0)
    {
        // 类似于 1 1 1 0 0 模式，可能需要左转
        if (one_time == 0)
        {
            one_time++;
            P = 0;
            I = 0;
            D = 0; // 重置PID
        }
        sensor_bias = -125; // 强左转
        decide = 4;
    }
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_Middle == 1 &&
             Sensor_MiddleRight == 1 && Sensor_Right == 1)
    {
        // 类似于 0 0 1 1 1 模式，可能需要右转
        sensor_bias = 125; // 强右转
        decide = 4;
        one_flag = 0;
    }
    // 正常偏差情况
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_Middle == 1 &&
             Sensor_MiddleRight == 0 && Sensor_Right == 0)
    {
        // 中间传感器检测到线，居中状态
        sensor_bias = 0;
        decide = 3;
        one_flag = 0; // 重置标记
        I = 0;        // 居中时清零积分项，防止累积
    }
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 1 && Sensor_Middle == 1 &&
             Sensor_MiddleRight == 0 && Sensor_Right == 0)
    {
        // 中左偏移
        sensor_bias = -62.5;
        decide = 2;
    }
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_Middle == 1 &&
             Sensor_MiddleRight == 1 && Sensor_Right == 0)
    {
        // 中右偏移
        sensor_bias = 62.5;
        decide = 2;
    }
    else if (Sensor_Left == 1 && Sensor_MiddleLeft == 1 && Sensor_Middle == 0 &&
             Sensor_MiddleRight == 0 && Sensor_Right == 0)
    {
        // 左侧强偏移
        sensor_bias = -125;
        decide = 4;
    }
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_Middle == 0 &&
             Sensor_MiddleRight == 1 && Sensor_Right == 1)
    {
        // 右侧强偏移
        sensor_bias = 125;
        decide = 4;
    }
    else if (Sensor_Left == 0 && Sensor_MiddleLeft == 0 && Sensor_Middle == 0 &&
             Sensor_MiddleRight == 0 && Sensor_Right == 0)
    {
        // 全白，可能偏离轨道
        decide = 6;
        // 保持上一次的转向方向，帮助寻找黑线
        if (sensor_bias_last < 0)
            sensor_bias = -125;
        else if (sensor_bias_last > 0)
            sensor_bias = 125;
    }
    // 其他组合
    else
    {
        // 根据亮起的传感器数量和位置计算加权偏差
        int weighted_sum = 0;
        int sensor_count = 0;

        if (Sensor_Left)
        {
            weighted_sum -= 200;
            sensor_count++;
        }
        if (Sensor_MiddleLeft)
        {
            weighted_sum -= 100;
            sensor_count++;
        }
        if (Sensor_Middle)
        {
            weighted_sum += 0;
            sensor_count++;
        }
        if (Sensor_MiddleRight)
        {
            weighted_sum += 100;
            sensor_count++;
        }
        if (Sensor_Right)
        {
            weighted_sum += 200;
            sensor_count++;
        }

        if (sensor_count > 0)
            sensor_bias = (float)weighted_sum / sensor_count;
        else
            sensor_bias = 0;

        decide = 5;
    }

    // 正常情况下进行PID计算
    if (decide <= 5)
    {
        P = sensor_bias;
        I = I + sensor_bias;
        D = sensor_bias - sensor_bias_last;

        // 积分限幅
        if (I >= INTEGRAL_LIMIT)
            I = INTEGRAL_LIMIT;
        if (I <= -INTEGRAL_LIMIT)
            I = -INTEGRAL_LIMIT;

        // PID计算，使用已经定义的全局PID参数
        PID_value = (Sensor_Kp / 100.0) * P + (Sensor_KI / 100.0) * I + (Sensor_Kd / 100.0) * D;

        // 保存上一次偏差
        sensor_bias_last = sensor_bias;
        return (int)PID_value;
    }
    else
    {
        // 特殊状态或全白/全黑状态
        return 0;
    }
}
/**
 * @brief  初始化红外传感器引脚
 * @param  无
 * @retval 无
 */
void Init_Sensor_Pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 启用GPIO时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 配置传感器1引脚 - 左传感器 */
    GPIO_InitStruct.Pin = SENSOR1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SENSOR1_PORT, &GPIO_InitStruct);

    /* 配置传感器2引脚 - 中左传感器 */
    GPIO_InitStruct.Pin = SENSOR2_PIN;
    HAL_GPIO_Init(SENSOR2_PORT, &GPIO_InitStruct);

    /* 配置传感器3引脚 - 中间传感器 */
    GPIO_InitStruct.Pin = SENSOR3_PIN;
    HAL_GPIO_Init(SENSOR3_PORT, &GPIO_InitStruct);

    /* 配置传感器4引脚 - 中右传感器 */
    GPIO_InitStruct.Pin = SENSOR4_PIN;
    HAL_GPIO_Init(SENSOR4_PORT, &GPIO_InitStruct);

    /* 配置传感器5引脚 - 右传感器 */
    GPIO_InitStruct.Pin = SENSOR5_PIN;
    HAL_GPIO_Init(SENSOR5_PORT, &GPIO_InitStruct);

#if FILTER_SAMPLES > 0
    /* 初始化滤波数组 */
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < FILTER_SAMPLES; j++)
        {
            sensor_history[i][j] = 0;
        }
    }

    /* 重置滤波索引 */
    filter_index = 0;
#endif
}
