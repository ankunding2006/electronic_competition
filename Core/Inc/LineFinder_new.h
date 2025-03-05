#ifndef __SENSOR_H
#define __SENSOR_H
 
//STEER4 --> PA11 --> R2  绿线
//STEER3 --> PC9 	--> R1  黄线
//		B  --> PB6  --> M0  橘线
//STEER1 --> PA6  --> L1  红线
// 		A  --> PB7	--> L2  白线
// 灰度传感器，当传感器识别到黑线的时候，输出为1，其余时刻输出为0
// 所以在这里我们要使用下拉输入
#define L2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)
#define L1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
#define M0 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define R1 GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)
#define R2 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)
 
void sensor_Init(void);
void sensor_read(void);
void Sensor_pid(void);
 
#endif