#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"
/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-01-04

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-01-04

All rights reserved
***********************************************/
#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�
void Encoder_Init_TIM2(void);             //��TIM2��ʼ��Ϊ�������ӿ�ģʽ
void Encoder_Init_TIM4(void);             //��TIM3��ʼ��Ϊ�������ӿ�ģʽ
int Read_Encoder(u8 TIMX);               //��λʱ���ȡ����������
#endif
