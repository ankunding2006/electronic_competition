#ifndef __USRAT2_H
#define __USRAT2_H 
#include "sys.h"	  	

extern u8 Usart2_Receive;
extern u8 Usart2_Receive_buf[1];//����2�����ж����ݴ�ŵĻ�����
void uart2_init(u32 bound);

#endif

