#include "usart2.h"
u8 Usart2_Receive;
u8 Usart2_Receive_buf[1];//����2�����ж����ݴ�ŵĻ�����
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
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //���ջص�����
{	
	if(UartHandle->Instance == USART2)//���յ�����
	{	  
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	    Usart2_Receive=Usart2_Receive_buf[0]; 
		APP_RX=Usart2_Receive;
		if(Usart2_Receive>=0x41&&Usart2_Receive<=0x48)  
		Flag_Direction=Usart2_Receive-0x40;
		else 	if(Usart2_Receive<10)  
		Flag_Direction=Usart2_Receive;	
		else 	if(Usart2_Receive==0X5A)  
		Flag_Direction=0;	
			
				//��������APP���Խ���ͨѶ
		if(Usart2_Receive==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
		if(Usart2_Receive==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ

		 if(Flag_PID==1)  //�ɼ�����
		 {
			Receive[i]=Usart2_Receive;
			i++;
		 }
		 if(Flag_PID==2)  //��������
		 {
			 if(Receive[3]==0x50) 	 PID_Send=1;
			 else  if(Receive[3]==0x57) 	 Flash_Send=1;
			 else  if(Receive[1]!=0x23) 
			 {								
				for(j=i;j>=4;j--)
				{
				  Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  Bluetooth_Velocity=Data;break;
					 case 0x31:  Velocity_KP=Data;break;
					 case 0x32:  Velocity_KI=Data;break;
					 case 0x33:  break;
					 case 0x34:  break;
					 case 0x35:  break;
					 case 0x36:  break;
					 case 0x37:  break; //Ԥ��
					 case 0x38:  break; //Ԥ��
				 }
				}				 
			 Flag_PID=0;//��ر�־λ����
			 i=0;
			 j=0;
			 Data=0;
			 memset(Receive, 0, sizeof(u8)*50);//��������
		 } 	 	
      HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//����2�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���		 
	}  											 
} 


