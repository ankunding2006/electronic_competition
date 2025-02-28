#ifndef __ELE_CCD_H
#define __ELE_CCD_H	
#include "sys.h"
#define Battery_Ch 9


void Dly_us(void);
 void RD_TSL(void) ;
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
void CCD(void);   
void CCD_Init(void);
void ELE_Init(void);
#endif 


