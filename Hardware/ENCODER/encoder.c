#include "encoder.h"
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

/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ���ٶ�ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
	 {
	   case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0; break;
	   case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0; break;	
	   default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
///**************************************************************************
//�������ܣ�TIM3�жϷ�����
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void TIM4_IRQHandler(void)//�жϴ�����Ϊ�գ�����жϱ�־λ������ж�
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//����ж�
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //����жϱ�־λ 	
//}
///**************************************************************************
//�������ܣ�TIM2�жϷ�����
//��ڲ�������
//����  ֵ����
//**************************************************************************/
//void TIM2_IRQHandler(void)//�жϴ�����Ϊ�գ�����жϱ�־λ������ж�
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)==SET)//����ж�
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //����жϱ�־λ 	  
//}
