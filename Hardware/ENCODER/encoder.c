#include "encoder.h"
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-01-04

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-01-04

All rights reserved
***********************************************/

/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
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
//函数功能：TIM3中断服务函数
//入口参数：无
//返回  值：无
//**************************************************************************/
//void TIM4_IRQHandler(void)//中断处理函数为空，清除中断标志位后结束中断
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//溢出中断
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位 	
//}
///**************************************************************************
//函数功能：TIM2中断服务函数
//入口参数：无
//返回  值：无
//**************************************************************************/
//void TIM2_IRQHandler(void)//中断处理函数为空，清除中断标志位后结束中断
//{ 		    		  			    
// if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)==SET)//溢出中断
// {
//	 
// } 
// TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //清除中断标志位 	  
//}
